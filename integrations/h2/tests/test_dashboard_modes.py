"""Exercise body-mode requests with a fake SDK; these tests never contact a robot."""
import importlib.machinery
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace
from urllib.request import Request, urlopen

source = Path(os.environ.get('H2_WEB_SOURCE', Path(__file__).resolve().parents[1] / 'deployment/robot/quest_h2_web.py'))
root = Path(os.environ.get('H2_DEPLOY_ROOT', Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(root))
loader = importlib.machinery.SourceFileLoader('dashboard_under_test', str(source))
spec = importlib.util.spec_from_loader(loader.name, loader)
web = importlib.util.module_from_spec(spec)
loader.exec_module(web)


class FakeSdk:
    def __init__(self, state=4, result=0):
        self.state, self.result, self.requests = state, result, []
        self.enable_calls = 0
        self.arm_sdk_enabled = True
        self.arm_sdk_code = 0

    def GetFsmId(self):
        return 0, self.state

    def SetFsmId(self, target):
        self.requests.append(target)
        if self.result == 0:
            self.state = target
        return self.result

    def EnableArmSDK(self):
        self.enable_calls += 1
        if self.result == 0:
            self.arm_sdk_enabled = True
        return self.result

    def GetArmSdkStatus(self):
        return self.arm_sdk_code, self.arm_sdk_enabled


def fsm(state=4, result=0):
    obj = web.Fsm.__new__(web.Fsm)
    obj.lock = threading.Lock()
    obj.client = FakeSdk(state, result)
    obj.fsm, obj.fsm_time, obj.error = state, time.monotonic(), ''
    obj.arm_sdk, obj.arm_sdk_time, obj.arm_sdk_error = True, time.monotonic(), ''
    obj.busy, obj.msg = False, ''
    return obj


class BodyModeTests(unittest.TestCase):
    def test_motion_requires_prepared_stand_and_stopped_teleop(self):
        for state in (0, 1, 703, None):
            with self.subTest(state=state):
                obj = fsm(state)
                if state is None:
                    obj.client.GetFsmId = lambda: (3102, None)
                ok, _ = obj.to_motion(False)
                self.assertFalse(ok)
                self.assertEqual(obj.client.requests, [])
        obj = fsm()
        self.assertFalse(obj.to_motion(True)[0])
        self.assertEqual(obj.client.requests, [])

    def test_only_explicit_request_schedules_transition_and_duplicate_is_rejected(self):
        obj = fsm()
        with patch.object(web.threading, 'Thread') as worker:
            self.assertTrue(obj.to_motion(False)[0])
            self.assertEqual(obj.client.requests, [])
            self.assertTrue(obj.busy)
            self.assertEqual(worker.call_args.kwargs['args'], ([601, 703],))
            worker.return_value.start.assert_called_once()
            self.assertFalse(obj.to_motion(False)[0])
            self.assertEqual(worker.call_count, 1)

    def test_transition_requests_only_normal_motion_and_checks_feedback(self):
        obj = fsm()
        obj.busy = True
        obj._transition([703])
        self.assertEqual(obj.client.requests, [703])
        self.assertEqual(obj.current(), 703)
        self.assertFalse(obj.busy)
        self.assertIn('Ready for walking', obj.msg)
        self.assertTrue(obj.walking_status()['ready'])
        self.assertEqual(obj.client.enable_calls, 1)

    def test_failed_request_does_not_retry_or_change_to_another_mode(self):
        obj = fsm(result=3102)
        obj.busy = True
        obj._transition([703])
        self.assertEqual(obj.client.requests, [703])
        self.assertEqual(obj.client.state, 4)
        self.assertFalse(obj.busy)
        self.assertIn('failed (3102)', obj.msg)

    def test_unconfirmed_request_times_out_without_claiming_success(self):
        obj = fsm()
        obj._read = Mock(return_value=False)
        obj.current = lambda: 4
        obj.busy = True
        with patch.object(web.time, 'monotonic', side_effect=[0, 11]):
            obj._transition([703])
        self.assertEqual(obj.client.requests, [703])
        self.assertFalse(obj.busy)
        self.assertIn('not confirmed', obj.msg)

    def test_body_mode_checks_running_under_same_lock_as_launch(self):
        up = web.Upstream.__new__(web.Upstream)
        up.lock = threading.Lock()
        up.fsm = fsm()
        def running():
            self.assertTrue(up.lock.locked())
            return True
        up.running = running
        self.assertFalse(up.body_mode(703)[0])
        self.assertEqual(up.fsm.client.requests, [])

    def test_previous_motion_mode_can_request_arm_compatible_mode(self):
        obj = fsm(601)
        with patch.object(web.threading, 'Thread') as worker:
            self.assertTrue(obj.to_motion(False)[0])
            self.assertEqual(worker.call_args.kwargs['args'], ([703],))
            self.assertEqual(obj.client.requests, [])

    def test_combined_walking_confirms_walk_run_before_requesting_arm_mode(self):
        obj = fsm()
        obj.busy = True
        obj._transition([601, 703])
        self.assertEqual(obj.client.requests, [601, 703])
        self.assertEqual(obj.client.enable_calls, 1)
        self.assertTrue(obj.walking_status()['ready'])

    def test_failed_walk_run_stage_does_not_request_arm_mode(self):
        obj = fsm(result=7302)
        obj.busy = True
        obj._transition([601, 703])
        self.assertEqual(obj.client.requests, [601])
        self.assertEqual(obj.client.enable_calls, 0)
        self.assertFalse(obj.walking_status()['ready'])

    def test_unconfirmed_walk_run_stage_does_not_request_arm_mode(self):
        obj = fsm()
        obj._read = Mock(return_value=False)
        obj.current = lambda: 4
        obj.busy = True
        with patch.object(web.time, 'monotonic', side_effect=[0, 11]):
            obj._transition([601, 703])
        self.assertEqual(obj.client.requests, [601])
        self.assertEqual(obj.client.enable_calls, 0)
        self.assertFalse(obj.busy)

    def test_launch_rejects_601_before_enabling_sdk_or_spawning(self):
        up = web.Upstream.__new__(web.Upstream)
        up.lock, up.fsm, up.running = threading.Lock(), fsm(601), lambda: False
        with patch.object(web.subprocess, 'Popen') as spawn:
            self.assertFalse(up.launch()[0])
            spawn.assert_not_called()
        self.assertEqual(up.fsm.client.enable_calls, 0)

    def test_launch_enables_sdk_in_both_supported_modes(self):
        import tempfile
        from types import SimpleNamespace
        for mode in (4, 703):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                up = web.Upstream.__new__(web.Upstream)
                up.lock, up.fsm, up.running = threading.Lock(), fsm(mode), lambda: False
                up.gain_store = web.GainStore(Path(directory) / 'dex5_gains.json')
                up.active_gains = None
                def spawn(*args, **kwargs):
                    self.assertEqual(up.fsm.client.enable_calls, 1)
                    self.assertIn('--motion', args[0])
                    return SimpleNamespace(pid=99999)
                with patch.object(web, 'LOGDIR', Path(directory)), patch.object(web.subprocess, 'Popen', side_effect=spawn):
                    self.assertTrue(up.launch()[0])

    def test_launch_failure_to_enable_sdk_does_not_spawn(self):
        up = web.Upstream.__new__(web.Upstream)
        up.lock, up.fsm, up.running = threading.Lock(), fsm(703, 3102), lambda: False
        up.gain_store = Mock(load=Mock(return_value=web.GainProfile()))
        with patch.object(web.subprocess, 'Popen') as spawn:
            self.assertFalse(up.launch()[0])
            spawn.assert_not_called()
        self.assertEqual(up.fsm.client.enable_calls, 1)


class WalkingReadinessTests(unittest.TestCase):
    def test_readiness_requires_both_mode_and_sdk_and_fresh_samples(self):
        for state in (0, 1, 4, 601, 703):
            for sdk in (True, False, None):
                with self.subTest(state=state, sdk=sdk):
                    obj = fsm(state)
                    obj.arm_sdk = sdk
                    self.assertEqual(obj.walking_status()['ready'], state == 703 and sdk is True)
        obj = fsm(703)
        obj.arm_sdk_time -= 4
        self.assertFalse(obj.walking_status()['ready'])
        obj.arm_sdk_time = time.monotonic()
        obj.fsm_time -= 4
        self.assertFalse(obj.walking_status()['ready'])
        obj.fsm_time = time.monotonic()
        obj.busy = True
        self.assertFalse(obj.walking_status()['ready'])

    def test_sdk_query_failure_does_not_display_ready(self):
        obj = fsm(703)
        obj.client.arm_sdk_code = 3102
        self.assertTrue(obj._read())
        self.assertFalse(obj.walking_status()['ready'])
        self.assertIsNone(obj.walking_status()['arm_sdk_enabled'])
        self.assertIn('3102', obj.arm_sdk_error)

    def test_invalid_sdk_reply_and_disconnection_clear_readiness(self):
        obj = fsm(703)
        obj.client.arm_sdk_enabled = 'true'
        obj._read()
        self.assertFalse(obj.walking_status()['ready'])
        obj.client.arm_sdk_enabled = True
        obj.client.GetFsmId = lambda: (3102, None)
        self.assertFalse(obj._read())
        self.assertIsNone(obj.current())
        self.assertFalse(obj.walking_status()['ready'])

    def test_existing_703_reenables_sdk_without_resetting_body_mode(self):
        obj = fsm(703)
        obj.client.arm_sdk_enabled = False
        self.assertTrue(obj._read())
        with patch.object(web.threading, 'Thread') as worker:
            self.assertTrue(obj.to_motion(False)[0])
            worker.return_value.start.assert_called_once()
        obj._transition([703])
        self.assertEqual(obj.client.requests, [])
        self.assertEqual(obj.client.enable_calls, 1)
        self.assertTrue(obj.walking_status()['ready'])

    def test_failed_arm_enable_never_claims_walking_ready(self):
        obj = fsm(703, 3102)
        obj.client.arm_sdk_enabled = False
        obj._read()
        obj.busy = True
        obj._transition([703])
        self.assertFalse(obj.walking_status()['ready'])
        self.assertIn('incomplete', obj.msg)
        self.assertEqual(obj.client.requests, [])

    def test_readiness_query_sends_no_motion_commands(self):
        obj = fsm(0)
        obj._read()
        obj.walking_status()
        self.assertEqual(obj.client.requests, [])
        self.assertEqual(obj.client.enable_calls, 0)


class WalkRunTests(unittest.TestCase):
    def test_explicit_walk_run_request_selects_601_from_supported_stand_or_walking(self):
        for state in (4, 703):
            with self.subTest(state=state):
                obj = fsm(state)
                with patch.object(web.threading, 'Thread') as worker:
                    self.assertTrue(obj.to_walk_run(False)[0])
                    self.assertEqual(worker.call_args.kwargs['args'], ([601],))
                    worker.return_value.start.assert_called_once()
                    self.assertEqual(obj.client.requests, [])
                    self.assertFalse(obj.to_walk_run(False)[0])
                    self.assertEqual(worker.call_count, 1)

    def test_running_unprepared_and_unavailable_states_send_no_request(self):
        for state, running in ((4, True), (0, False), (1, False), (None, False)):
            with self.subTest(state=state, running=running):
                obj = fsm(state)
                if state is None:
                    obj.client.GetFsmId = lambda: (3102, None)
                self.assertFalse(obj.to_walk_run(running)[0])
                self.assertEqual(obj.client.requests, [])
                self.assertEqual(obj.client.enable_calls, 0)

    def test_already_active_does_not_reissue_mode_or_enable_arm_sdk(self):
        obj = fsm(601)
        ok, message = obj.to_walk_run(False)
        self.assertFalse(ok)
        self.assertIn('already active', message)
        self.assertEqual(obj.client.requests, [])
        self.assertEqual(obj.client.enable_calls, 0)

    def test_601_confirmation_never_claims_arm_teleop_ready(self):
        obj = fsm()
        obj.busy = True
        obj._transition([601])
        self.assertEqual(obj.client.requests, [601])
        self.assertEqual(obj.client.enable_calls, 0)
        self.assertFalse(obj.busy)
        self.assertIn('Walk/Run confirmed', obj.msg)
        self.assertTrue(obj.walking_status()['walk_run_active'])
        self.assertFalse(obj.walking_status()['ready'])
        self.assertFalse(obj.walking_status()['mode_confirmed'])
        obj.fsm_time -= 4
        self.assertFalse(obj.walking_status()['walk_run_active'])

    def test_failed_walk_run_request_does_not_try_another_mode(self):
        obj = fsm(result=7302)
        obj.busy = True
        obj._transition([601])
        self.assertEqual(obj.client.requests, [601])
        self.assertEqual(obj.client.state, 4)
        self.assertFalse(obj.walking_status()['walk_run_active'])
        self.assertEqual(obj.client.enable_calls, 0)

    def test_timeout_reports_actual_mode_without_retrying(self):
        obj = fsm()
        obj._read = Mock(return_value=False)
        obj.current = lambda: 4
        obj.busy = True
        with patch.object(web.time, 'monotonic', side_effect=[0, 11]):
            obj._transition([601])
        self.assertEqual(obj.client.requests, [601])
        self.assertIn('reported FSM 4', obj.msg)
        self.assertFalse(obj.busy)

    def test_http_routes_keep_walk_run_and_arm_walking_distinct(self):
        up = web.Upstream.__new__(web.Upstream)
        up.body_mode = Mock(return_value=(True, 'scheduled'))
        recorder = SimpleNamespace(start=lambda: (True, ''), stop=lambda: (True, ''))
        server = web.ThreadingHTTPServer(('127.0.0.1', 0), web.handler(up, recorder))
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            base = f'http://127.0.0.1:{server.server_port}'
            for route, target in (('walk-run', 601), ('motion', 703)):
                with urlopen(Request(base + '/api/' + route, method='POST'), timeout=3) as response:
                    self.assertEqual(response.status, 200)
                    self.assertEqual(json.load(response)['message'], 'scheduled')
                up.body_mode.assert_called_with(target)
        finally:
            server.shutdown()
            server.server_close()
            worker.join(timeout=2)


if __name__ == '__main__':
    unittest.main()
