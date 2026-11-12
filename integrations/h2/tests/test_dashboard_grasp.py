"""Standalone grip API/Quest conflict checks using only a fake worker manager."""
from contextlib import nullcontext
import json
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from test_dashboard_gains import make_up, serve, web
import tempfile
from pathlib import Path


class DashboardGraspTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.up = make_up(Path(self.temp.name) / 'gains.json')
        self.up.grasp = Mock()
        self.up.grasp.teleop_slot.side_effect = nullcontext
        self.up.grasp.status.return_value = {'active': True, 'phase': 'holding', 'message': 'Holding on H2'}

    def test_launch_during_hold_never_enables_arms_or_spawns(self):
        with patch.object(web.subprocess, 'Popen') as spawn:
            ok, message = self.up.launch()
        self.assertFalse(ok)
        self.assertIn('Release', message)
        spawn.assert_not_called()
        self.up.fsm.enable_arm_sdk.assert_not_called()

    def test_status_reattaches_to_existing_worker_without_commands(self):
        status = self.up.status()
        self.assertEqual(status['right_grasp']['phase'], 'holding')
        self.up.grasp.start.assert_not_called()
        self.up.grasp.release.assert_not_called()

    def test_http_buttons_invoke_grasp_and_release_without_body_or_quest_commands(self):
        self.up.grasp.start.return_value = (True, 'Closing')
        self.up.grasp.release.return_value = (True, 'Opening')
        with serve(self.up) as base, patch.object(web.subprocess, 'Popen') as spawn:
            for endpoint in ('grasp-hold', 'grasp-release'):
                with urlopen(Request(base+'/api/'+endpoint, method='POST'), timeout=3) as response:
                    self.assertEqual(response.status, 200)
                    json.load(response)
            self.up.grasp.start.assert_called_once_with(self.up.running)
            self.up.grasp.release.assert_called_once_with()
            self.up.fsm.enable_arm_sdk.assert_not_called()
            spawn.assert_not_called()

    def test_rejected_grasp_is_conflict_response(self):
        self.up.grasp.start.return_value = (False, 'Stop Quest teleop first')
        with serve(self.up) as base:
            with self.assertRaises(HTTPError) as caught:
                urlopen(Request(base+'/api/grasp-hold', method='POST'), timeout=3)
            self.assertEqual(caught.exception.code, 409)

    def test_dashboard_shutdown_does_not_release_or_stop_grasp(self):
        self.up.close()
        self.up.grasp.release.assert_not_called()
        self.up.grasp.start.assert_not_called()


if __name__ == '__main__':
    unittest.main()
