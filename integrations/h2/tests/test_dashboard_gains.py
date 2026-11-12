"""HTTP persistence and launch-time gain handoff, using no robot or DDS session."""
from contextlib import contextmanager
from html.parser import HTMLParser
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
spec = importlib.util.spec_from_file_location('gain_dashboard_under_test', ROOT / 'deployment/robot/quest_h2_web.py')
web = importlib.util.module_from_spec(spec)
spec.loader.exec_module(web)


def make_up(path):
    up = web.Upstream.__new__(web.Upstream)
    up.lock = threading.Lock()
    up.gain_store = web.GainStore(path)
    up.active_gains = None
    up.process = None
    up.last_exit = None
    up.log_path = None
    up.hb, up.hb_time, up.started_at = {}, 0.0, None
    up.external_pids = lambda: []
    up.fsm = SimpleNamespace(current=lambda: 703, busy=False, msg='', error='',
                             walking_status=lambda: {'ready': True, 'message': 'Ready for walking',
                                                     'arm_sdk_enabled': True, 'mode_confirmed': True},
                             enable_arm_sdk=Mock(return_value=(True, 'enabled')))
    return up


@contextmanager
def serve(up):
    server = web.ThreadingHTTPServer(('127.0.0.1', 0), web.handler(up, SimpleNamespace(
        status=lambda: {}, start=lambda: (True, 'fake recorder'), stop=lambda: (True, 'fake recorder'))))
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    try:
        yield f'http://127.0.0.1:{server.server_port}'
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=2)


def post(base, values):
    request = Request(base + '/api/hand-gains', json.dumps(values).encode(),
                      headers={'Content-Type': 'application/json'}, method='POST')
    try:
        with urlopen(request, timeout=3) as response:
            return response.status, json.load(response)
    except HTTPError as error:
        return error.code, json.load(error)


class DashboardGainsTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / 'dex5_gains.json'
        self.up = make_up(self.path)

    def test_extended_range_is_selectable_and_persists(self):
        class Sliders(HTMLParser):
            def __init__(self):
                super().__init__()
                self.inputs = {}

            def handle_starttag(self, tag, attrs):
                attrs = dict(attrs)
                if tag == 'input' and attrs.get('type') == 'range':
                    self.inputs[attrs['id']] = attrs

        with serve(self.up) as base:
            with urlopen(base + web.PAGE_PATH, timeout=3) as response:
                page = Sliders()
                page.feed(response.read().decode())
            for name in ('finger-gain', 'thumb-gain'):
                slider = page.inputs[name]
                self.assertEqual(float(slider['min']), web.MIN_MULTIPLIER)
                self.assertEqual(float(slider['max']), 2.5)
                self.assertEqual(float(slider['max']), web.MAX_MULTIPLIER)
                self.assertEqual(float(slider['step']), .05)
                self.assertEqual(float(slider['value']), 1.2)
            for finger, thumb in ((2.25, 2.5), (2.5, 2.25), (2.5, 2.5)):
                with self.subTest(finger=finger, thumb=thumb):
                    values = {'finger_multiplier': finger, 'thumb_multiplier': thumb}
                    self.assertEqual(post(base, values)[0], 200)
                    restarted = make_up(self.path)
                    self.assertEqual(restarted.gain_store.load().settings(), values)
                    self.assertEqual(restarted.hand_gain_status(False)['max'], 2.5)

    def test_save_and_reset_persist_across_dashboard_instances(self):
        with serve(self.up) as base:
            self.assertEqual(post(base, {'finger_multiplier': 1.05, 'thumb_multiplier': 1.2})[0], 200)
            restarted = make_up(self.path)
            self.assertEqual(restarted.hand_gain_status(False)['saved']['finger_multiplier'], 1.05)
            self.assertEqual(post(base, {'finger_multiplier': 1, 'thumb_multiplier': 1})[0], 200)
            with urlopen(base + '/api/status', timeout=3) as response:
                settings = json.load(response)['hand_gains']['saved']
            self.assertAlmostEqual(settings['finger_kp'], .1)
            self.assertAlmostEqual(settings['finger_kd'], .001)
            self.assertAlmostEqual(settings['thumb_kp'], 1)
            self.assertAlmostEqual(settings['thumb_kd'], .02)

    def test_invalid_requests_do_not_overwrite_saved_settings(self):
        self.up.gain_store.save(web.GainProfile())
        original = self.path.read_bytes()
        invalid = [[], {}, {'finger_multiplier': 1},
                   {'finger_multiplier': True, 'thumb_multiplier': 1},
                   {'finger_multiplier': '1', 'thumb_multiplier': 1},
                   {'finger_multiplier': float('nan'), 'thumb_multiplier': 1},
                   {'finger_multiplier': float('inf'), 'thumb_multiplier': 1},
                   {'finger_multiplier': .49, 'thumb_multiplier': 1},
                   {'finger_multiplier': 2.5001, 'thumb_multiplier': 1},
                   {'finger_multiplier': 1, 'thumb_multiplier': 2.5001},
                   {'finger_multiplier': 3, 'thumb_multiplier': 3},
                   {'finger_multiplier': 1, 'thumb_multiplier': 1, 'force': 100}]
        with serve(self.up) as base:
            for values in invalid:
                with self.subTest(values=values):
                    self.assertEqual(post(base, values)[0], 400)
                    self.assertEqual(self.path.read_bytes(), original)

    def test_save_during_teleop_is_pending_and_keeps_active_profile(self):
        self.up.active_gains = web.GainProfile()
        self.up.process = Mock(poll=Mock(return_value=None))
        with serve(self.up) as base:
            self.assertEqual(post(base, {'finger_multiplier': 1, 'thumb_multiplier': 1.1})[0], 200)
            with urlopen(base + '/api/status', timeout=3) as response:
                gains = json.load(response)['hand_gains']
        self.assertTrue(gains['pending'])
        self.assertEqual(gains['active']['finger_multiplier'], 1.2)
        self.assertEqual(gains['saved']['finger_multiplier'], 1)
        self.up.fsm.enable_arm_sdk.assert_not_called()

    def test_launch_sends_saved_profile_to_new_process(self):
        profile = web.GainProfile(2.25, 2.5)
        self.up.gain_store.save(profile)
        with patch.object(web, 'LOGDIR', Path(self.directory.name)), patch.object(web.subprocess, 'Popen') as spawn:
            self.assertTrue(self.up.launch()[0])
            env = spawn.call_args.kwargs['env']
        # The actual launcher environment reaches a clean child interpreter and the
        # same gain resolver used by the Dex5 hand controller, without importing DDS.
        result = subprocess.run([sys.executable, '-c',
            'import json; from teleop.utils.dex5_gains import GainProfile; '
            'print(json.dumps(GainProfile.from_environment().describe()))'],
            cwd=self.directory.name, env=env, capture_output=True, text=True, check=True)
        self.assertEqual(json.loads(result.stdout), profile.describe())
        self.assertEqual(self.up.active_gains, profile)
        self.up.gain_store.save(web.GainProfile(1, 1))
        self.assertEqual(self.up.active_gains, profile)

    def test_invalid_file_prevents_launch_before_sdk_enable(self):
        self.path.write_text('{invalid')
        with patch.object(web.subprocess, 'Popen') as spawn:
            ok, message = self.up.launch()
            self.assertFalse(ok)
            self.assertIn('could not be loaded', message)
            spawn.assert_not_called()
        self.up.fsm.enable_arm_sdk.assert_not_called()
        self.assertTrue(self.up.hand_gain_status(False)['error'])

    def test_failed_atomic_save_preserves_previous_file(self):
        self.up.gain_store.save(web.GainProfile())
        original = self.path.read_bytes()
        with patch('teleop.utils.dex5_gains.os.replace', side_effect=OSError('disk unavailable')):
            with self.assertRaises(OSError):
                self.up.gain_store.save(web.GainProfile(1, 1))
        self.assertEqual(self.path.read_bytes(), original)
        self.assertEqual(list(self.path.parent.iterdir()), [self.path])

    def test_fingers_and_thumb_are_independent(self):
        profile = web.GainProfile(1.2, 1)
        self.assertAlmostEqual(profile.finger_kp, .12)
        self.assertAlmostEqual(profile.finger_kd, .0011)
        self.assertAlmostEqual(profile.thumb_kp, 1)
        self.assertAlmostEqual(profile.thumb_kd, .02)


if __name__ == '__main__':
    unittest.main()
