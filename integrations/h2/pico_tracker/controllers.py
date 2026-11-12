"""Official XRoboToolkit controller poses, independent of optical hand input."""
import math

from .geometry import pose_valid

CONTROLLER_SOURCE = 'pico_controller_wuji'
CONTROLLER_IDS = {side: 'pico-controller-'+side for side in ('left', 'right')}


def read_controllers(sdk, previous, changed, now):
    """Keep pose-change age visible: the SDK has no controller tracking flag."""
    result = {}
    for side, identity in CONTROLLER_IDS.items():
        value = {'pose': None, 'pose_age_ms': None}
        try:
            pose = list(getattr(sdk, 'get_'+side+'_controller_pose')())
            if pose_valid(pose):
                if pose != previous.get(identity):
                    changed[identity] = now
                previous[identity] = pose
                value = {'pose': [round(x, 6) for x in pose],
                         'pose_age_ms': round((now-changed[identity])*1000, 1)}
        except (AttributeError, TypeError, ValueError, RuntimeError):
            pass
        result[side] = value
    return result


def controller_config():
    # The existing START reference absorbs the controller's mounting rotation.
    # Physical wrist-tracker assignments and mounting offsets stay separate.
    return {side+'_sn': identity for side, identity in CONTROLLER_IDS.items()}


def controller_frame(frame):
    """Adapt controller roles to the existing wrist mapper, without fallback."""
    if frame is None:
        return None
    values = frame.get('controllers', {})
    if not isinstance(values, dict):
        values = {}
    poses, ages = {}, {}
    for side, identity in CONTROLLER_IDS.items():
        value = values.get(side)
        if not isinstance(value, dict) or not pose_valid(value.get('pose')):
            continue
        age = value.get('pose_age_ms')
        if not isinstance(age, (int, float)) or not math.isfinite(age) or age < 0:
            continue
        poses[identity], ages[identity] = value['pose'], age
    return dict(frame, trackers=poses, pose_age_ms=ages, wrist_source='controllers')
