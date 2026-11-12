"""Native PICO optical fingers using Unitree's existing Dex5 controller.

Importing this module creates no SDK connection or command publisher.
"""
import math
import threading

from .geometry import matvec, pose_valid, rotation, transpose

# T_TO_UNITREE_HAND @ T_ROBOT_OPENXR from the pinned Unitree tv_wrapper.py.
# The input is expressed in the optical wrist's OpenXR frame, independently
# of the physical wrist tracker used by arm_teleop.py.
UNITREE_HAND_FROM_OPENXR = [[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]]


def valid_landmarks(points):
    return (isinstance(points, (list, tuple)) and len(points) == 25 and
            all(isinstance(p, (list, tuple)) and len(p) == 3 and
                all(isinstance(x, (int, float)) and math.isfinite(x) for x in p)
                for p in points) and any(abs(x) > 1e-6 for p in points for x in p))


def hand_landmarks(joints):
    """Convert OpenXR's 26 joints to Unitree's wrist-relative 25 landmarks."""
    if not isinstance(joints, (list, tuple)) or len(joints) != 26:
        raise ValueError('Expected the 26 PICO/OpenXR hand joints')
    wrist = joints[1]  # OpenXR: palm=0, wrist=1; WebXR starts at wrist.
    if not pose_valid(wrist):
        raise ValueError('Optical wrist pose unavailable')
    inverse_rotation = transpose(rotation(wrist))
    result = []
    for joint in joints[1:]:
        if (not isinstance(joint, (list, tuple)) or len(joint) != 7 or
                not all(isinstance(x, (int, float)) and math.isfinite(x) for x in joint[:3])):
            raise ValueError('Invalid optical finger position')
        local = matvec(inverse_rotation, [joint[i]-wrist[i] for i in range(3)])
        # Micrometre precision keeps the complete input frame within PIPE_BUF,
        # preserving the existing atomic, latest-sample transport.
        result.append([round(x, 6) for x in matvec(UNITREE_HAND_FROM_OPENXR, local)])
    if not valid_landmarks(result):
        raise ValueError('Optical finger landmarks unavailable')
    return result


def read_hands(sdk):
    hands = {}
    for side in ('left', 'right'):
        value = {'active': False, 'landmarks': None}
        try:
            if getattr(sdk, f'get_{side}_hand_is_active')():
                value['landmarks'] = hand_landmarks(getattr(sdk, f'get_{side}_hand_tracking_state')())
                value['active'] = True
        except (AttributeError, ValueError, TypeError, IndexError, RuntimeError):
            pass
        hands[side] = value
    return hands


def hands_status(frame):
    hands = frame.get('hands', {}) if frame else {}
    if not isinstance(hands, dict):
        hands = {}
    hands = {side: value if isinstance(value, dict) else {}
             for side, value in hands.items()}
    sides = {side: bool(hands.get(side, {}).get('active') and
                       valid_landmarks(hands.get(side, {}).get('landmarks')))
             for side in ('left', 'right')}
    missing = [side for side, ready in sides.items() if not ready]
    return {'ready': not missing, 'sides': sides,
            'message': ('PICO fingers ready' if not missing else
                        'Enable Hand in XRoboToolkit and show '+', '.join(missing)+' hand(s)')}


class OpticalFingers:
    """Feed the unchanged Unitree Dex5 controller; never supplies arm targets."""
    def __init__(self, controller_factory=None):
        from multiprocessing import Array, Value, active_children
        if controller_factory is None:
            from teleop.robot_control.robot_hand_unitree import Dex5_1_Controller
            controller_factory = Dex5_1_Controller
        self.arrays = {side: Array('d', 75, lock=True) for side in ('left', 'right')}
        self.enabled = Value('b', False)
        self.lock = threading.RLock()
        self.session = None
        before = {p.pid for p in active_children()}
        self.controller = controller_factory(self.arrays['left'], self.arrays['right'],
                                            xr_motion_data_ready_in=self.enabled)
        self.children = [p for p in active_children() if p.pid not in before]

    def start(self, frame):
        with self.lock:
            status = hands_status(frame)
            if not status['ready']:
                raise ValueError(status['message'])
            self.session = frame['session']
            return self.update(frame)

    def update(self, frame):
        with self.lock:
            same_session = bool(frame and self.session is not None and frame.get('session') == self.session)
            status = hands_status(frame if same_session else None)
            if same_session:
                for side, ready in status['sides'].items():
                    if ready:
                        points = frame['hands'][side]['landmarks']
                        with self.arrays[side].get_lock():
                            self.arrays[side][:] = [x for p in points for x in p]
            with self.enabled.get_lock():
                # A lost side retains its own last landmarks. The other side keeps
                # updating through the official dual-hand retargeting controller.
                self.enabled.value = same_session and any(status['sides'].values())
            return status

    def stop(self):
        with self.lock:
            with self.enabled.get_lock():
                self.enabled.value = False
            self.session = None

    def close(self):
        self.stop()
        for child in self.children:
            if child.is_alive():
                child.terminate()
            child.join(timeout=.5)
            if child.is_alive():
                # The official child may inherit the arm process's signal
                # handlers. Bound shutdown so its publisher cannot outlive STOP.
                child.kill()
                child.join(timeout=2)
