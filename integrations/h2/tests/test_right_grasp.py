"""Right-only hold, process independence and explicit release; no real DDS/motors."""
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import quest_right_grasp as grasp
from quest_h2_teleop import UPSTREAM
from teleop.utils.dex5_gains import GainProfile, GainStore


class GraspTests(unittest.TestCase):
    def test_targets_respect_urdf_and_include_every_digit(self):
        robot = ET.parse(UPSTREAM / 'assets/unitree_hand_Dex5/Dex5-URDF-R.urdf')
        limits = {j.attrib['name']: (float(j.find('limit').attrib['lower']), float(j.find('limit').attrib['upper']))
                  for j in robot.findall('joint') if j.find('limit') is not None}
        self.assertEqual(len(grasp.FULL_GRASP), 20)
        for pose in (grasp.FULL_GRASP, grasp.OPEN_HAND):
            for name, q in zip(grasp.JOINT_NAMES, pose):
                low, high = limits[name]
                self.assertTrue(low <= q <= high, (name, q, low, high))
        for start in (0, 4, 8, 12):
            self.assertTrue(all(q > 1 for q in grasp.FULL_GRASP[start+1:start+4]))
        self.assertTrue(all(q > 1 for q in grasp.FULL_GRASP[17:20]))

    def test_hold_is_target_based_and_does_not_expire_without_commands(self):
        motion = grasp.GraspMotion([0.] * 20, 0.)
        self.assertEqual(motion.step(0.), (0.,) * 20)
        half = motion.step(1.5)
        self.assertTrue(all(abs(a) <= abs(b) for a,b in zip(half, grasp.FULL_GRASP)))
        for now in (3., 30., 600., 86400.):
            self.assertEqual(motion.step(now), grasp.FULL_GRASP)
            self.assertEqual(motion.phase, 'holding')
        self.assertEqual(motion.step(86401., release=True), grasp.FULL_GRASP)
        self.assertEqual(motion.phase, 'releasing')
        self.assertEqual(motion.step(86404., release=True), grasp.OPEN_HAND)
        self.assertEqual(motion.phase, 'released')

    def test_release_mid_close_starts_from_current_command(self):
        motion = grasp.GraspMotion([0.] * 20, 0.)
        before = motion.step(1.)
        self.assertEqual(motion.step(1., release=True), before)
        self.assertEqual(motion.step(4., release=True), grasp.OPEN_HAND)

    def test_import_or_manager_creation_never_starts_a_process(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(grasp.subprocess, 'Popen') as spawn:
            manager = grasp.GraspManager(Mock(), directory=directory)
            self.assertFalse(manager.status()['active'])
            self.assertFalse(manager.release()[0])
            spawn.assert_not_called()

    def test_stale_state_after_worker_exit_cannot_auto_reactivate(self):
        with tempfile.TemporaryDirectory() as directory:
            grasp.atomic_json(Path(directory) / 'status.json', {'phase': 'holding', 'updated': time.time(), 'session': 'old'})
            manager = grasp.GraspManager(Mock(), directory=directory)
            self.assertFalse(manager.status()['active'])
            self.assertEqual(manager.status()['phase'], 'idle')

    def test_second_writer_is_rejected_and_unknown_worker_stays_active(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = grasp.GraspManager(Mock(), directory=directory)
            with (Path(directory) / 'worker.lock').open('a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX)
                with patch.object(grasp, 'quest_pids', return_value=[]), patch.object(grasp.subprocess, 'Popen') as spawn:
                    self.assertFalse(manager.start(lambda: False)[0])
                    spawn.assert_not_called()
                self.assertTrue(manager.status()['active'])
                self.assertEqual(manager.status()['phase'], 'unresponsive')

    def test_quest_conflict_rejected_without_spawning(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(grasp, 'quest_pids', return_value=[123]), patch.object(grasp.subprocess, 'Popen') as spawn:
            manager = grasp.GraspManager(Mock(), directory=directory)
            self.assertFalse(manager.start(lambda: False)[0])
            spawn.assert_not_called()

    def test_missing_feedback_sends_no_commands_and_reports_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            dds = Mock()
            dds.feedback.return_value = (None, 100., None)
            args = SimpleNamespace(directory=directory, session='test', interface='eth0')
            ticks = iter(range(20))
            with patch.object(grasp, 'quest_pids', return_value=[]), patch.object(grasp.signal, 'signal'), patch.object(grasp.time, 'sleep'), patch.object(grasp.time, 'monotonic', side_effect=lambda: next(ticks)):
                result = grasp.run_worker(args, GainProfile(), dds_factory=lambda *_: dds)
            self.assertEqual(result, 1)
            dds.write.assert_not_called()
            status = grasp.GraspManager(Mock(), directory=directory).status()
            self.assertFalse(status['active'])
            self.assertEqual(status['phase'], 'error')
            self.assertIn('No fresh right-hand feedback', status['message'])

    def test_dds_adapter_creates_only_right_topics_and_uses_saved_gains(self):
        from unitree_sdk2py.core import channel
        publisher, subscriber = Mock(), Mock()
        profile = GainProfile(2., 2.5)
        with patch.object(channel, 'ChannelFactoryInitialize') as initialize, patch.object(channel, 'ChannelPublisher', return_value=publisher) as pub, patch.object(channel, 'ChannelSubscriber', return_value=subscriber) as sub:
            dds = grasp.RightDds('eth0', profile)
            initialize.assert_called_once_with(0, 'eth0')
            self.assertEqual(pub.call_args.args[0], 'rt/dex5/right/cmd')
            self.assertEqual(sub.call_args.args[0], 'rt/dex5/right/state')
            pub.assert_called_once()
            sub.assert_called_once()
            dds.write(grasp.FULL_GRASP)
            msg = publisher.Write.call_args.args[0]
            self.assertEqual(tuple(c.q for c in msg.motor_cmd), grasp.FULL_GRASP)
            for index, cmd in enumerate(msg.motor_cmd):
                self.assertEqual(cmd.mode, 1)
                self.assertEqual(cmd.tau, 0.)
                self.assertEqual(cmd.dq, 0.)
                self.assertEqual(cmd.kp, profile.thumb_kp if index >= 16 else profile.finger_kp)
            publisher.Write.return_value = False
            with self.assertRaises(RuntimeError):
                dds.write(grasp.FULL_GRASP)

    def test_independent_process_holds_without_dashboard_then_reattaches_and_releases(self):
        # Child only uses this fake DDS adapter. No channel factory/network exists.
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            driver = directory / 'fake_worker.py'
            driver.write_text('''import json, sys, time
from pathlib import Path
sys.path.insert(0, ROOT)
import quest_right_grasp as g
class FakeDds:
    def __init__(self, interface, profile):
        self.q=(0.,)*20
        self.log=Path(sys.argv[sys.argv.index('--directory')+1])/'commands.jsonl'
    def feedback(self): return self.q, 0., [0,0]
    def write(self, q):
        self.q=tuple(q)
        with self.log.open('a') as f: f.write(json.dumps(list(q))+'\\n')
sys.exit(g.main(dds_factory=FakeDds))
'''.replace('ROOT', repr(str(ROOT))))
            store = GainStore(directory / 'gains.json')
            store.save(GainProfile(1.5, 2.))
            manager = grasp.GraspManager(store, directory=directory)
            real_popen, children = subprocess.Popen, []
            def spawn(args, **kwargs):
                self.assertTrue(kwargs['start_new_session'])
                child = real_popen([args[0], str(driver)] + args[2:], **kwargs)
                children.append(child)
                return child
            try:
                with patch.object(grasp, 'quest_pids', return_value=[]), patch.object(grasp.subprocess, 'Popen', side_effect=spawn):
                    self.assertTrue(manager.start(lambda: False)[0])
                child = children[0]
                state = manager.status()
                self.assertEqual(state['gains']['finger_multiplier'], 1.5)
                self.assertEqual(state['gains']['thumb_multiplier'], 2.)
                # Old requests from another session must not open the new grasp.
                grasp.atomic_json(directory / 'request.json', {'session': 'old', 'action': 'release'})
                del manager
                # No HTTP/dashboard/Quest instance is alive while the worker closes.
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    state = grasp.read_json(directory / 'status.json')
                    if state.get('phase') == 'holding': break
                    time.sleep(.1)
                self.assertEqual(state['phase'], 'holding')
                count = state['writes']
                time.sleep(.6)
                reconnected = grasp.GraspManager(store, directory=directory)
                self.assertTrue(reconnected.status()['active'])
                self.assertEqual(reconnected.status()['phase'], 'holding')
                self.assertGreater(reconnected.status()['writes'], count)
                self.assertTrue(reconnected.release()[0])
                self.assertEqual(child.wait(timeout=6), 0)
                self.assertFalse(reconnected.status()['active'])
                self.assertTrue(reconnected.status()['open_confirmed'])
                commands = [json.loads(line) for line in (directory/'commands.jsonl').read_text().splitlines()]
                self.assertEqual(commands[-1], list(grasp.OPEN_HAND))
                self.assertTrue(any(q == list(grasp.FULL_GRASP) for q in commands))
            finally:
                for child in children:
                    if child.poll() is None:
                        child.terminate()
                        child.wait(timeout=3)


if __name__ == '__main__':
    unittest.main()
