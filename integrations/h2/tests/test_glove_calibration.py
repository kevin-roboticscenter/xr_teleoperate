"""Calibration lifecycle and browser API using a simulated official CLI."""
import io
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch
from http.server import ThreadingHTTPServer
from urllib.request import Request, urlopen
from urllib.error import HTTPError

from wuji_glove.calibration import CalibrationManager, CalibrationError
from wuji_glove import dashboard_gateway as gateway

SN = 'GLOVE_TEST_LEFT'


def event(kind, **values):
    return dict(schema_version=2, operation='calibrate', calibration='hand_model', event=kind, **values)


def success(user='operator', side='left', sn=SN):
    return event('result', status='ok', device={'sn':sn}, user={'display_name':user},
                 result={'handedness':side, 'sdk_user':{'display_name':user},
                         'calibrated_urdf':'/private-test-profile/models/'+side+'_hand.urdf',
                         'frames_per_pose':{'flat_open':120}, 'quality_grade':'B'})


class FakeCLI:
    def __init__(self):
        self.name = 'operator'
        self.calibrated = True
        self.side = 'left'
        self.fail_backup = False
        self.calls = []

    def run(self, argv, **kwargs):
        self.calls.append(list(argv))
        args = argv[1:-1]
        code = 0
        if args == ['user','list']:
            data = {'users':[{'name':'Default','is_default':True,'is_current':self.name=='Default'},
                             {'name':self.name,'is_current':True,'calibrated':{'left':self.calibrated,'right':False}}]}
        elif args == ['user','show']:
            data = {'name':self.name,'left_hand':{'calibrated':self.calibrated},'right_hand':{'calibrated':False}}
        elif args == ['devices']:
            data = {'devices':[{'sn':SN,'device_type':'wuji_glove','transport':'Udp'},
                               {'sn':'OTHER_HAND','device_type':'wuji_hand_2'}]}
        elif args[:2] == ['get','hand_side']:
            data = {'value':self.side}
        elif args[:2] == ['user','export']:
            if self.fail_backup:
                code,data = 1,{'error':{'code':1,'message':'Backup failed'}}
            else:
                Path(args[2]).write_bytes(b'test backup')
                data = {'path':args[2]}
        elif args[:2] in (['user','create'],['user','switch']):
            self.name = args[2]
            data = {'current_user':self.name}
        else:
            raise AssertionError('Unexpected CLI operation: '+str(argv))
        return subprocess.CompletedProcess(argv,code,json.dumps(data),'')


class FakeProcess:
    def __init__(self, events, code=0, block=False):
        self.events, self.code, self.block = events,code,block
        self.returncode = None
        self.signals = []
        self.cancelled = threading.Event()
        self.streaming = threading.Event()
        self.stdout = self.Output(self)

    class Output:
        def __init__(self, process):self.process,self.closed=process,False
        def __iter__(self):
            p = self.process
            p.streaming.set()
            if p.block:
                yield json.dumps(event('progress',user={'display_name':'operator'},feedback={'step_index':0,'step_total':6,'step_name':'pinch_index','state':'waiting_movement','progress':0.}))+'\n'
                if not p.cancelled.wait(3):raise AssertionError('Test did not cancel its calibration')
                p.code = 9
                yield json.dumps(event('cancelled',status='cancelled'))+'\n'
            else:
                for item in p.events:yield json.dumps(item)+'\n'
        def close(self):self.closed=True

    def poll(self):return self.returncode
    def wait(self):self.returncode=self.code;return self.code
    def send_signal(self,value):self.signals.append(value);self.cancelled.set()
    def communicate(self):self.wait();return ('','')


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.cli = FakeCLI()
        self.disconnect = Mock()
        self.idle = Mock()
        self.process = FakeProcess([success()])
        self.spawn = Mock(side_effect=lambda *a,**k:self.process)
        self.manager = CalibrationManager(self.root,self.disconnect,self.idle,
                                          run=self.cli.run,popen=self.spawn,state_root=self.root/'private-state')
        self.payload = {'profile':'operator','side':'left','sn':SN,'ready':True,'overwrite':True,'assign':True}

    def tearDown(self):
        self.manager.close()
        self.temporary.cleanup()

    def finish(self):
        self.manager.worker.join(timeout=4)
        self.assertFalse(self.manager.worker.is_alive())
        return self.manager.status()

    def test_success_backs_up_confirms_identity_and_assigns_only_selected_hand(self):
        self.manager.start(self.payload)
        result = self.finish()
        self.assertEqual(result['phase'],'succeeded')
        self.assertEqual(result['exit_code'],0)
        self.assertTrue(Path(result['backup']).is_file())
        self.assertEqual(json.loads((self.root/'config/wuji_gloves.json').read_text()),{'left':SN})
        self.assertEqual(self.spawn.call_args.args[0],['wuji','calib','hand-model','--sn',SN,'--jsonl'])
        self.disconnect.assert_called_once()
        self.assertEqual(result['result']['sdk_user']['display_name'],'operator')

    def test_missing_overwrite_consent_keeps_existing_inputs_and_never_starts_calibration(self):
        self.manager.start(dict(self.payload,overwrite=False))
        self.assertEqual(self.finish()['phase'],'failed')
        self.spawn.assert_not_called()
        self.disconnect.assert_not_called()
        self.assertFalse(any(c[1:3]==['user','export'] for c in self.cli.calls))

    def test_default_user_is_rejected_before_device_or_profile_changes(self):
        self.cli.name = 'Default'
        self.manager.start(dict(self.payload,profile='Default'))
        self.assertEqual(self.finish()['phase'],'failed')
        self.spawn.assert_not_called()
        self.disconnect.assert_not_called()
        self.assertEqual(len(self.cli.calls),1)

    def test_glove_reported_side_must_match_the_selected_hand(self):
        self.cli.side = 'right'
        self.manager.start(self.payload)
        self.assertIn('reports right',self.finish()['message'])
        self.spawn.assert_not_called()

    def test_online_robot_following_blocks_calibration(self):
        self.idle.side_effect = CalibrationError('Stop teleoperation first')
        self.manager.start(self.payload)
        self.assertEqual(self.finish()['message'],'Stop teleoperation first')
        self.disconnect.assert_not_called()
        self.spawn.assert_not_called()

    def test_backup_failure_leaves_calibration_unstarted(self):
        self.cli.fail_backup = True
        self.manager.start(self.payload)
        self.assertEqual(self.finish()['message'],'Backup failed')
        self.spawn.assert_not_called()

    def test_success_exit_without_terminal_result_is_not_success(self):
        self.process = FakeProcess([])
        self.manager.start(self.payload)
        result = self.finish()
        self.assertEqual(result['phase'],'failed')
        self.assertIn('without a terminal result',result['message'])

    def test_terminal_error_preserves_failure_code_and_does_not_retry(self):
        self.process = FakeProcess([event('error',status='failed',error={'kind':'default_user','code':5,'message':'Default cannot calibrate'})],code=5)
        self.manager.start(self.payload)
        result = self.finish()
        self.assertEqual(result['phase'],'failed')
        self.assertEqual(result['exit_code'],5)
        self.spawn.assert_called_once()

    def test_wrong_result_owner_is_not_reported_or_assigned_as_success(self):
        self.process = FakeProcess([success(user='another-operator')])
        self.manager.start(self.payload)
        self.assertEqual(self.finish()['phase'],'failed')
        self.assertFalse((self.root/'config/wuji_gloves.json').exists())

    def test_cancel_sends_one_interrupt_and_excludes_other_cli_commands_until_exit(self):
        self.process = FakeProcess([],block=True)
        self.manager.setup(scan=True)
        self.manager.start(self.payload)
        self.assertTrue(self.process.streaming.wait(2))
        count = len(self.cli.calls)
        self.manager.setup(scan=True)
        self.assertEqual(len(self.cli.calls),count)
        with self.assertRaises(CalibrationError):self.manager.profile('switch','another-operator')
        with self.assertRaises(CalibrationError):self.manager.start(self.payload)
        self.manager.cancel();self.manager.cancel()
        result = self.finish()
        self.assertEqual(self.process.signals,[signal.SIGINT])
        self.assertEqual(result['phase'],'cancelled')
        self.assertEqual(result['exit_code'],9)
        self.spawn.assert_called_once()

    def test_missing_physical_readiness_starts_nothing(self):
        with self.assertRaises(CalibrationError):self.manager.start(dict(self.payload,ready=False))
        self.spawn.assert_not_called()
        self.assertEqual(self.cli.calls,[])

    def test_profile_selection_uses_only_official_sdk_profile_commands(self):
        self.manager.profile('create','New operator')
        self.assertEqual(self.cli.calls[-1],['wuji','user','create','New operator','--switch','--json'])
        self.manager.profile('switch','operator')
        self.assertEqual(self.cli.calls[-1],['wuji','user','switch','operator','--json'])
        with self.assertRaises(CalibrationError):self.manager.profile('delete','operator')

    def test_setup_lists_only_gloves_and_preserves_current_models(self):
        setup = self.manager.setup(scan=True)
        self.assertEqual(setup['current']['name'],'operator')
        self.assertEqual([d['sn'] for d in setup['devices']],[SN])
        self.assertTrue(setup['current']['left_hand']['calibrated'])
        self.disconnect.assert_not_called()
        self.spawn.assert_not_called()

    def test_using_saved_calibration_assigns_glove_without_recalibrating(self):
        self.manager.assign(self.payload)
        self.assertEqual(json.loads((self.root/'config/wuji_gloves.json').read_text()),{'left':SN})
        self.spawn.assert_not_called()

    def test_page_and_setup_work_without_robot_access_and_reject_cross_origin_start(self):
        server = ThreadingHTTPServer(('127.0.0.1',0),gateway.handler(Mock(),calibration=self.manager))
        thread = threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        base = f'http://127.0.0.1:{server.server_port}'
        try:
            with patch.object(gateway,'HTTPConnection',side_effect=AssertionError('Robot access is unexpected')):
                with urlopen(base+'/wuji-calibration') as r:self.assertIn(b'Wuji glove calibration',r.read())
                with urlopen(base+'/api/wuji/calibration/setup') as r:self.assertEqual(json.load(r)['current']['name'],'operator')
                req = Request(base+'/api/wuji/calibration/start',data=json.dumps(self.payload).encode(),
                              headers={'Content-Type':'application/json','Origin':'https://unrelated.example'},method='POST')
                with self.assertRaises(HTTPError) as error:urlopen(req)
                self.assertEqual(error.exception.code,403)
            self.spawn.assert_not_called()
        finally:server.shutdown();server.server_close();thread.join(timeout=2)

    def test_browser_start_cancel_and_teleop_exclusion_use_the_managed_job(self):
        self.process = FakeProcess([],block=True)
        server = ThreadingHTTPServer(('127.0.0.1',0),gateway.handler(Mock(),calibration=self.manager))
        thread = threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        base = f'http://127.0.0.1:{server.server_port}'
        def post(path,data):
            return urlopen(Request(base+path,data=json.dumps(data).encode(),
                                   headers={'Content-Type':'application/json'},method='POST'),timeout=3)
        try:
            with post('/api/wuji/calibration/start',self.payload) as r:self.assertEqual(r.status,202)
            self.assertTrue(self.process.streaming.wait(2))
            with patch.object(gateway,'HTTPConnection',side_effect=AssertionError('Teleop must remain blocked')):
                with self.assertRaises(HTTPError) as error:post('/api/start',{})
                self.assertEqual(error.exception.code,409)
            with post('/api/wuji/calibration/cancel',{}) as r:self.assertEqual(r.status,200)
            self.assertEqual(self.finish()['phase'],'cancelled')
        finally:server.shutdown();server.server_close();thread.join(timeout=2)

    def test_hand_side_verification_does_not_guess_or_calibrate_an_unknown_value(self):
        self.cli.side = 'unknown'
        self.manager.start(self.payload)
        self.assertIn('valid hand side',self.finish()['message'])
        self.spawn.assert_not_called()
