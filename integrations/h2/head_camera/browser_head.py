"""Head following for the pinned Unitree browser loop, using its existing writer.

Hooks are installed only by browser_teleop.py when Follow headset is selected.
Arm/hand observations and the upstream controller/IK sources stay unchanged.
"""
import math
from multiprocessing import Value
import threading
import time

from .head_follow import HeadPoseMapper, HeadCommandPublisher
from pico_tracker.geometry import pose_valid


def matrix_pose(matrix):
    """OpenXR column-major CAMERA_MOVE matrix, or its 4x4 shared-array view."""
    if hasattr(matrix, 'tolist'):
        matrix = matrix.tolist()
    if isinstance(matrix, (list, tuple)) and len(matrix) == 16:
        matrix = [[matrix[4*c+r] for c in range(4)] for r in range(4)]
    if (not isinstance(matrix, (list, tuple)) or len(matrix) != 4 or
            any(not isinstance(row, (list, tuple)) or len(row) != 4 for row in matrix) or
            any(not isinstance(v, (int, float)) or not math.isfinite(v) for row in matrix for v in row)):
        raise ValueError('Browser headset pose unavailable')
    r = [row[:3] for row in matrix[:3]]
    for i in range(3):
        for j in range(3):
            if abs(sum(r[i][k]*r[j][k] for k in range(3))-(1. if i == j else 0.)) > .01:
                raise ValueError('Browser headset rotation unavailable')
    determinant = (r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1])-
                   r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0])+
                   r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0]))
    if abs(determinant-1.) > .01 or any(abs(matrix[3][i]-(1. if i == 3 else 0.)) > .01 for i in range(4)):
        raise ValueError('Invalid browser headset transform')
    trace = sum(r[i][i] for i in range(3))
    if trace > 0:
        s = 2*math.sqrt(trace+1.)
        q = [(r[2][1]-r[1][2])/s, (r[0][2]-r[2][0])/s, (r[1][0]-r[0][1])/s, .25*s]
    else:
        i = max(range(3), key=lambda n: r[n][n])
        j, k = (i+1)%3, (i+2)%3
        s = 2*math.sqrt(max(0., 1.+r[i][i]-r[j][j]-r[k][k]))
        q = [0.]*4
        q[i], q[j], q[k], q[3] = .25*s, (r[j][i]+r[i][j])/s, (r[k][i]+r[i][k])/s, (r[k][j]-r[j][k])/s
    pose = [matrix[i][3] for i in range(3)]+q
    if not pose_valid(pose):
        raise ValueError('Browser headset quaternion unavailable')
    return pose


class BrowserHeadBridge:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.RLock()
        self.source = self.publisher = self.neck_reader = self.mapper = None
        self.active, self.target = False, None

    def frame(self):
        try:
            matrix, stamp = self.source()
            if not stamp or not 0 <= self.clock()-stamp < .5:
                return None
            return matrix_pose(matrix)
        except (TypeError, ValueError, AttributeError, OSError, RuntimeError, IndexError):
            return None

    def attach_controller(self, controller):
        neck_reader = lambda: [float(v) for v in controller.get_current_motor_q()[29:31]]
        publisher = HeadCommandPublisher(controller.lowcmd_publisher, controller.crc, neck_reader())
        controller.lowcmd_publisher = publisher
        self.publisher, self.neck_reader = publisher, neck_reader

    def start(self):
        with self.lock:
            pose = self.frame()
            if pose is None or self.publisher is None:
                raise ValueError('Enter the headset browser XR view and enable live head tracking before START')
            neck = self.neck_reader()
            self.mapper = HeadPoseMapper(pose, neck)
            self.publisher.set_target(neck)
            self.target, self.active = neck, True

    def stop(self):
        with self.lock:
            self.active = False

    def recenter(self):
        if not self.active:
            raise ValueError('Start head following first')
        self.start()

    def update(self):
        with self.lock:
            pose = self.frame()
            if self.active and pose is not None:
                self.target = list(self.mapper.target(pose))
                self.publisher.set_target(self.target)

    def state(self):
        with self.lock:
            ready = self.frame() is not None and self.publisher is not None
            return {'HEAD_FOLLOW': True, 'HEAD_READY': ready, 'HEAD_LIVE': bool(self.active and ready),
                    'HEAD_TARGET': self.target}


def install_hooks(bridge, arm_module, ipc_module, tv_module, tv_package):
    """Wrap integration boundaries in-process; return a restoration callback."""
    original_arm = arm_module.H2_ArmController
    original_ipc = ipc_module.IPC_Server
    original_tv = tv_module.TeleVuer
    original_wrapper = tv_package.TeleVuerWrapper

    class TimedTeleVuer(original_tv):
        def __init__(self, *args, **kwargs):
            self.head_received_mono = Value('d', 0.)
            super().__init__(*args, **kwargs)

        async def on_cam_move(self, event, session, fps=60):
            try:
                matrix_pose(event.value['camera']['matrix'])
            except (KeyError, TypeError, ValueError):
                self.head_received_mono.value = 0.
                return
            await super().on_cam_move(event, session, fps)
            self.head_received_mono.value = time.monotonic()

    class HeadWrapper(original_wrapper):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            bridge.source = lambda: (self.tvuer.head_pose, self.tvuer.head_received_mono.value)

        def get_tele_data(self, *args, **kwargs):
            data = super().get_tele_data(*args, **kwargs)
            bridge.update()
            return data

    class HeadArm(original_arm):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            bridge.attach_controller(self)

    class HeadIPC(original_ipc):
        def __init__(self, on_press=None, get_state=None, *args, **kwargs):
            def press(key):
                if key == 'h':
                    return bridge.recenter()
                if key == 'r':
                    bridge.start()
                elif key == 'q':
                    bridge.stop()
                try:
                    return on_press(key)
                except Exception:
                    if key == 'r': bridge.stop()
                    raise

            def state():
                return dict(get_state(), **bridge.state())

            super().__init__(*args, on_press=press, get_state=state, **kwargs)
            self.cmd_map = dict(self.cmd_map, CMD_HEAD_RECENTER='h')

    arm_module.H2_ArmController = HeadArm
    ipc_module.IPC_Server = HeadIPC
    tv_module.TeleVuer = TimedTeleVuer
    tv_package.TeleVuerWrapper = HeadWrapper

    def restore():
        bridge.stop()
        arm_module.H2_ArmController = original_arm
        ipc_module.IPC_Server = original_ipc
        tv_module.TeleVuer = original_tv
        tv_package.TeleVuerWrapper = original_wrapper
    return restore
