"""Start alignment and deferred following, using fake inputs/controllers only."""
import copy
import json
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch
from http.server import ThreadingHTTPServer
from urllib.request import Request, urlopen

from pico_tracker.geometry import rotation, matvec
from pico_tracker.pose_match import PoseReference, MatchDwell, matching_enabled
from test_hybrid_hands import controller, web

CONFIG = {'left_sn': 'TEST_LEFT', 'right_sn': 'TEST_RIGHT',
          'offset_tracker_m': [.01, 0., 0.], 'right_offset_tracker_m': [0., .02, 0.]}
SIDES = ('left', 'right')


def sample():
    return {'session': 'input-a', 'seq': 1, 'head_pose': [0., 1.6, 0., 0., 0., 0., 1.],
            'trackers': {'TEST_LEFT': [-.3, .8, -.3, 0., 0., 0., 1.],
                         'TEST_RIGHT': [.3, .8, -.3, 0., 0., 0., 1.]}}


def wrists():
    return {s: [[1., 0., 0., .28], [0., 1., 0., .18 if s == 'left' else -.18],
                [0., 0., 1., .17], [0., 0., 0., 1.]] for s in SIDES}


class ReferenceGeometry(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.reference = PoseReference(self.temp.name)
        self.frame, self.robot = sample(), wrists()

    def save(self, sides=SIDES):
        return self.reference.save(self.frame, self.robot, CONFIG, sides)

    def assess(self, frame=None, robot=None, config=None, sides=SIDES):
        return self.reference.assess(self.frame if frame is None else frame,
                                     self.robot if robot is None else robot,
                                     CONFIG if config is None else config, sides)

    def test_first_start_needs_a_taught_reference_and_read_does_not_create_it(self):
        result = self.assess()
        self.assertFalse(result['calibrated'])
        self.assertFalse(result['ready'])
        self.assertFalse(self.reference.path.exists())
        self.save()
        self.assertTrue(self.assess()['ready'])
        self.assertEqual(self.reference.path.stat().st_mode & 0o777, 0o600)

    def test_room_translation_and_yaw_do_not_require_recalibration(self):
        self.save()
        moved = sample()
        q = [0., math.sin(.7), 0., math.cos(.7)]
        r = rotation([0., 0., 0.]+q)
        for pose in [moved['head_pose'], *moved['trackers'].values()]:
            p = matvec(r, pose[:3])
            pose[:] = [p[0]+3., p[1]+.4, p[2]-2.]+q
        self.assertTrue(self.assess(frame=moved)['ready'])

    def test_one_wrong_wrist_blocks_both_and_reports_position_guidance(self):
        self.save()
        self.frame['trackers']['TEST_LEFT'][0] += .07
        result = self.assess()
        self.assertFalse(result['ready'])
        self.assertFalse(result['sides']['left']['matched'])
        self.assertTrue(result['sides']['right']['matched'])
        self.assertAlmostEqual(result['sides']['left']['position_error_m'], .07)
        self.assertAlmostEqual(result['sides']['left']['move_m'][1], .07)

    def test_wrist_orientation_is_checked_even_with_correct_position(self):
        self.save()
        self.frame['trackers']['TEST_RIGHT'][3:] = [0., math.sin(math.radians(10)), 0., math.cos(math.radians(10))]
        result = self.assess()
        self.assertFalse(result['ready'])
        self.assertAlmostEqual(result['sides']['right']['orientation_error_deg'], 20., places=1)

    def test_robot_must_still_be_in_the_taught_ready_pose(self):
        self.save()
        self.robot['left'][0][3] += .04
        result = self.assess()
        self.assertTrue(result['calibrated'])
        self.assertFalse(result['robot_matches'])
        self.assertFalse(result['ready'])

    def test_serial_swap_or_new_mounting_offset_invalidates_reference(self):
        self.save()
        for key, value in [('left_sn', 'TEST_RIGHT'), ('offset_tracker_m', [.02, 0., 0.])]:
            config = dict(CONFIG, **{key: value})
            self.assertFalse(self.assess(config=config)['calibrated'])

    def test_dual_reference_can_be_used_left_only_but_left_only_cannot_authorize_both(self):
        self.save()
        self.assertTrue(self.assess(sides=('left',))['ready'])
        self.save(sides=('left',))
        self.assertFalse(self.assess()['calibrated'])

    def test_missing_tracking_does_not_erase_a_valid_calibration(self):
        self.save()
        result = self.reference.assess(None, self.robot, CONFIG, SIDES)
        self.assertTrue(result['calibrated'])
        self.assertFalse(result['ready'])
        del self.frame['trackers']['TEST_LEFT']
        self.assertFalse(self.assess()['ready'])

    def test_corrupt_reference_and_nonfinite_robot_feedback_cannot_match(self):
        value = self.save()
        value['sides']['left']['human']['rotation'][0][0] = float('nan')
        self.reference.path.write_text(json.dumps(value))
        self.assertFalse(self.assess()['calibrated'])
        self.save()
        self.robot['left'][0][3] = float('nan')
        self.assertFalse(self.assess()['ready'])

    def test_saving_without_live_input_fails_without_overwriting(self):
        self.save()
        before = self.reference.path.read_bytes()
        with self.assertRaises(ValueError):
            self.reference.save(None, self.robot, CONFIG, SIDES)
        self.assertEqual(self.reference.path.read_bytes(), before)


def assessment(now, **values):
    return dict({'calibrated': True, 'ready': True, 'robot_matches': True,
                 'reference_id': 'reference-a', 'input_session': 'input-a',
                 'evaluated_mono': now, 'message': 'Both wrists match'}, **values)


class ContinuousMatch(unittest.TestCase):
    def test_full_second_is_required_after_start_and_failure_restarts_the_timer(self):
        dwell = MatchDwell()
        self.assertFalse(dwell.update(assessment(0.), True, 0.)['complete'])
        self.assertFalse(dwell.update(assessment(.9), True, .9)['complete'])
        self.assertFalse(dwell.update(assessment(1., ready=False), True, 1.)['complete'])
        self.assertFalse(dwell.update(assessment(1.1), True, 1.1)['complete'])
        self.assertTrue(dwell.update(assessment(2.2), True, 2.2)['complete'])

    def test_stale_heartbeat_or_missing_fingers_cannot_complete_a_match(self):
        for lost in ('stale', 'fingers'):
            dwell = MatchDwell()
            dwell.update(assessment(0.), True, 0.)
            self.assertFalse(dwell.update(assessment(0. if lost == 'stale' else 1.1), lost != 'fingers', 1.1)['complete'])
            self.assertFalse(dwell.update(assessment(1.2), True, 1.2)['complete'])

    def test_new_input_session_or_reference_requires_a_new_full_match(self):
        for field in ('input_session', 'reference_id'):
            dwell = MatchDwell()
            dwell.update(assessment(0.), True, 0.)
            self.assertFalse(dwell.update(assessment(1.1, **{field: 'changed'}), True, 1.1)['complete'])


def prepared(source='pico_dual_wuji', ipc_ok=True):
    up, events = controller(source, ipc_ok=ipc_ok)
    up.hb['POSE_MATCH'] = assessment(time.monotonic())
    up.fsm = Mock(busy=False)
    up.fsm.current.return_value = 4
    up.tracker = Mock()
    up.tracker.status.return_value = {'ready': True, 'message': 'Tracking ready'}
    up.running = lambda: True
    up.auto_recording = Mock()
    return up, events


def arm_wait(up):
    with patch.object(web.threading, 'Thread') as thread:
        ok, _ = up.start()
        if ok: thread.return_value.start.assert_called_once()
    return ok


def match_tick(up, dwell, now, **changes):
    up.hb['POSE_MATCH'] = assessment(now, **changes)
    with up.lock:
        return up._match_tick(dwell, now)


class DeferredFollowing(unittest.TestCase):
    def test_start_waits_and_neither_gloves_arms_nor_recording_start_before_match(self):
        up, events = prepared()
        self.assertTrue(arm_wait(up))
        self.assertEqual(events, [])
        up.auto_recording.notify.assert_not_called()
        self.assertTrue(up.pose_waiting)
        dwell = MatchDwell()
        self.assertIsNone(match_tick(up, dwell, 0.))
        self.assertIsNone(match_tick(up, dwell, .9))
        self.assertEqual(events, [])
        self.assertEqual(match_tick(up, dwell, 1.1), (True, 'Following started'))
        self.assertEqual(events, ['glove_start', 'CMD_START'])
        up.auto_recording.notify.assert_called_once()

    def test_optical_fingers_use_the_same_wait_without_starting_any_glove(self):
        up, events = prepared('pico_tracker_hands')
        self.assertTrue(arm_wait(up))
        dwell = MatchDwell()
        match_tick(up, dwell, 0.)
        match_tick(up, dwell, 1.1)
        self.assertEqual(events, ['CMD_START'])

    def test_missing_calibration_blocks_start_and_start_cannot_arm_twice(self):
        up, events = prepared()
        up.hb['POSE_MATCH'] = {}
        self.assertFalse(arm_wait(up))
        self.assertEqual(events, [])
        up.hb['POSE_MATCH'] = assessment(time.monotonic())
        self.assertTrue(arm_wait(up))
        self.assertFalse(up.start()[0])
        self.assertEqual(events, [])

    def test_stop_cancels_the_wait_and_an_old_worker_cannot_start_later(self):
        up, events = prepared()
        self.assertTrue(arm_wait(up))
        generation = up.pose_wait_generation
        self.assertTrue(up.stop()[0])
        up._wait_for_pose(generation)
        self.assertFalse(up.pose_waiting)
        self.assertEqual(events, ['glove_stop', 'CMD_STOP'])
        up.auto_recording.notify.assert_not_called()

    def test_missing_input_resets_the_wait_and_no_body_mode_is_requested(self):
        up, events = prepared()
        self.assertTrue(arm_wait(up))
        dwell = MatchDwell()
        match_tick(up, dwell, 0.)
        up.tracker.status.return_value = {'ready': False, 'message': 'Show left hand'}
        self.assertIsNone(match_tick(up, dwell, 1.1))
        self.assertEqual(up.pose_wait_message, 'Show left hand')
        self.assertEqual(events, [])
        up.tracker.status.return_value = {'ready': True, 'message': 'Tracking ready'}
        self.assertIsNone(match_tick(up, dwell, 1.2))
        match_tick(up, dwell, 2.3)
        self.assertEqual(events, ['glove_start', 'CMD_START'])
        self.assertEqual(set(c[0] for c in up.fsm.method_calls), {'current'})

    def test_native_process_rejection_stops_the_new_glove_publisher(self):
        up, events = prepared(ipc_ok=False)
        self.assertTrue(arm_wait(up))
        dwell = MatchDwell()
        match_tick(up, dwell, 0.)
        self.assertFalse(match_tick(up, dwell, 1.1)[0])
        self.assertEqual(events, ['glove_start', 'CMD_START', 'glove_stop'])
        self.assertFalse(up.pose_waiting)
        up.auto_recording.notify.assert_not_called()

    def test_calibration_only_uses_its_explicit_command_before_waiting_or_following(self):
        up, events = prepared()
        self.assertTrue(up.save_matching_pose()[0])
        self.assertEqual(events, ['CMD_POSE_CALIBRATE'])
        self.assertTrue(arm_wait(up))
        self.assertFalse(up.save_matching_pose()[0])
        self.assertEqual(events, ['CMD_POSE_CALIBRATE'])

    def test_actual_worker_starts_once_after_a_continuous_match(self):
        up, events = prepared('pico_tracker_hands')
        with patch('pico_tracker.pose_match.HOLD_SECONDS', .05):
            self.assertTrue(up.start()[0])
            deadline = time.monotonic()+.4
            while up.pose_waiting and time.monotonic()<deadline:
                time.sleep(.01)
        self.assertFalse(up.pose_waiting)
        self.assertEqual(events, ['CMD_START'])

    def test_dashboard_calibration_route_dispatches_only_the_teach_command(self):
        up, events = prepared()
        server = ThreadingHTTPServer(('127.0.0.1', 0), web.handler(up, Mock()))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            request = Request(f'http://127.0.0.1:{server.server_port}/api/pico/pose-calibrate', data=b'', method='POST')
            with urlopen(request, timeout=3) as response:
                self.assertEqual(response.status, 200)
                self.assertIn('Matching pose saved', json.load(response)['message'])
            self.assertEqual(events, ['CMD_POSE_CALIBRATE'])
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


class OptionalMatching(unittest.TestCase):
    def test_disabled_setting_preserves_the_taught_reference(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            reference=PoseReference(root)
            reference.save(sample(),wrists(),CONFIG,SIDES)
            before=reference.path.read_bytes()
            self.assertTrue(matching_enabled(root))
            (root/'config/pico_pose_match_settings.json').write_text('{"enabled":false}')
            self.assertFalse(matching_enabled(root))
            self.assertEqual(reference.path.read_bytes(),before)

    def test_disabled_matching_starts_directly_but_still_requires_live_input(self):
        up,events=prepared('pico_tracker_hands')
        up.hb.pop('POSE_MATCH')
        with patch.object(web,'matching_enabled',return_value=False):
            self.assertFalse(up.pose_match_status(up.hb)['enabled'])
            up.tracker.status.return_value={'ready':False,'message':'Show hands'}
            self.assertFalse(up.start()[0]);self.assertEqual(events,[])
            up.tracker.status.return_value={'ready':True,'message':'Ready'}
            self.assertTrue(up.start()[0])
            self.assertEqual(events,['CMD_START'])

    def test_disabling_a_pending_check_does_not_automatically_start(self):
        up,events=prepared()
        with patch.object(web,'matching_enabled',return_value=True):self.assertTrue(arm_wait(up))
        with patch.object(web,'matching_enabled',return_value=False):up._wait_for_pose(up.pose_wait_generation)
        self.assertFalse(up.pose_waiting)
        self.assertEqual(events,[])


if __name__ == '__main__':
    unittest.main()
