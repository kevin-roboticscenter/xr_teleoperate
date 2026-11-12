"""Production HTTP wiring with fake processes only; no START reaches a robot."""
from contextlib import contextmanager
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from test_dashboard_gains import make_up, web
from test_auto_recording import AutoRecording, FakeRecorder, FOLLOWING


class DashboardAutoRecordingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.up = make_up(Path(self.temp.name) / 'gains.json')
        self.rec = FakeRecorder()
        self.auto = AutoRecording(self.rec, self.up.recording_snapshot,
                                  Path(self.temp.name) / 'auto.json', start_worker=False)
        self.up.auto_recording = self.auto

    @contextmanager
    def serve(self):
        server = web.ThreadingHTTPServer(('127.0.0.1', 0), web.handler(self.up, self.rec, self.auto))
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            yield f'http://127.0.0.1:{server.server_port}'
        finally:
            server.shutdown()
            server.server_close()
            worker.join(2)

    def test_http_settings_and_manual_stop_route_to_automation(self):
        with self.serve() as base:
            request = Request(base+'/api/auto-record', json.dumps({'enabled':False}).encode(),
                              {'Content-Type':'application/json'}, method='POST')
            with urlopen(request) as response:
                self.assertEqual(response.status, 200)
            self.assertFalse(self.auto.status()['enabled'])
            self.auto.set_enabled(True)
            self.auto.update(FOLLOWING)
            with urlopen(Request(base+'/api/rec-stop', method='POST')) as response:
                self.assertEqual(response.status, 200)
            self.assertTrue(self.auto.status()['paused'])
            with urlopen(base+'/api/status') as response:
                state=json.load(response)
            self.assertFalse(state['rec']['recording'])
            self.assertTrue(state['rec']['auto']['paused'])

    def test_invalid_settings_do_not_change_enabled_state(self):
        with self.serve() as base:
            for value in ({'enabled':'true'}, {'enabled':1}, {'enabled':False,'extra':True}, []):
                request=Request(base+'/api/auto-record',json.dumps(value).encode(),
                                {'Content-Type':'application/json'},method='POST')
                with self.assertRaises(HTTPError) as error:
                    urlopen(request)
                self.assertEqual(error.exception.code,400)
            self.assertTrue(self.auto.status()['enabled'])

    def test_accepted_start_wakes_recording_worker_and_failed_start_does_not(self):
        self.up.hb={'READY':True,'START':False,'HEAD_READY':True}
        self.up.hb_time=time.monotonic()
        self.up._send=Mock(return_value=(False,'test rejection'))
        self.up.auto_recording=Mock()
        self.assertFalse(self.up.start()[0])
        self.up.auto_recording.notify.assert_not_called()
        self.up._send.return_value=(True,'started')
        self.assertTrue(self.up.start()[0])
        self.up.auto_recording.notify.assert_called_once_with()

    def test_snapshot_reports_pid_and_fresh_following_flag(self):
        self.up.external_pids=lambda:[42]
        self.up.hb={'START':True}
        self.up.hb_time=time.monotonic()
        snapshot=self.up.recording_snapshot()
        self.assertEqual(snapshot,FOLLOWING)
        self.up.hb_time-=5
        self.assertFalse(self.up.recording_snapshot()['started'])

    def test_recording_segment_names_are_unique_and_share_camera_streams(self):
        recorder=web.Recorder('eth0')
        with patch.object(web,'RECDIR',Path(self.temp.name)), patch.object(web,'LOGDIR',Path(self.temp.name)), patch.object(web.subprocess,'Popen',return_value=SimpleNamespace(poll=lambda:0)) as spawn:
            recorder.start();first=recorder.dir
            recorder.start();second=recorder.dir
        self.assertNotEqual(first,second)
        self.assertIn('--camera-streams',spawn.call_args.args[0])
        self.assertIn('--head-camera',spawn.call_args.args[0])


if __name__ == '__main__':
    unittest.main()
