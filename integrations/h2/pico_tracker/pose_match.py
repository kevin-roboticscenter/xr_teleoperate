"""A taught wrist reference and a continuous match before native PICO START.

No SDK, DDS or motor access. A person must teach the reference while physically
matching the prepared robot. Head-relative coordinates remove room origin/yaw;
they do not infer elbow angles or automatically calibrate tracker mounting.
"""
import json
import math
from pathlib import Path
import time
import uuid

from .geometry import heading_basis, matmul, matvec, pose_valid, rotation, transpose

POSITION_M = .05
ORIENTATION_DEG = 15.
ROBOT_POSITION_M = .025
ROBOT_ORIENTATION_DEG = 10.
HOLD_SECONDS = 1.
ASSESSMENT_MAX_AGE = .35


def distance(a, b):
    return math.sqrt(sum((x-y)**2 for x, y in zip(a, b)))


def angle_degrees(a, b):
    relative = matmul(a, transpose(b))
    return math.degrees(math.acos(max(-1., min(1., (sum(relative[i][i] for i in range(3))-1.)/2.))))


def wrist_in_head_frame(frame, serial, offset):
    head = frame['head_pose']
    tracker = frame['trackers'][serial]
    if not pose_valid(head) or not pose_valid(tracker):
        raise ValueError('Head and wrist tracker poses must be valid')
    basis = heading_basis(head)
    r = rotation(tracker)
    lever = matvec(r, offset)
    p = matvec(basis, [tracker[i]+lever[i]-head[i] for i in range(3)])
    return {'position': p, 'rotation': matmul(basis, r)}


def tracker_identity(config, side):
    offset = config.get('offset_tracker_m' if side == 'left' else 'right_offset_tracker_m', [0., 0., 0.])
    if (not isinstance(offset, (list, tuple)) or len(offset) != 3 or
            not all(isinstance(x, (int, float)) and math.isfinite(x) for x in offset)):
        raise ValueError('Invalid tracker mounting offset')
    serial = config.get(side+'_sn')
    if not isinstance(serial, str) or not serial:
        raise ValueError(f'Choose the {side} wrist tracker')
    return {'serial': serial, 'offset': list(offset)}


def robot_wrist(transform):
    if (not isinstance(transform, (list, tuple)) or len(transform) != 4 or
            any(not isinstance(row, (list, tuple)) or len(row) != 4 for row in transform) or
            any(not isinstance(x, (int, float)) or not math.isfinite(x) for row in transform for x in row)):
        raise ValueError('Robot wrist feedback unavailable')
    return {'position': [transform[i][3] for i in range(3)],
            'rotation': [list(row[:3]) for row in transform[:3]]}


def valid_reference_pose(value):
    if not isinstance(value, dict):
        return False
    p, r = value.get('position'), value.get('rotation')
    if (not isinstance(p, list) or len(p) != 3 or
            not isinstance(r, list) or len(r) != 3 or
            any(not isinstance(row, list) or len(row) != 3 for row in r)):
        return False
    if any(not isinstance(x, (int, float)) or not math.isfinite(x) for x in p+[x for row in r for x in row]):
        return False
    product = matmul(r, transpose(r))
    return all(abs(product[i][j]-(1. if i == j else 0.)) < .01 for i in range(3) for j in range(3))


class PoseReference:
    def __init__(self, root):
        self.path = Path(root)/'config/pico_pose_match.json'

    def save(self, frame, robot_poses, config, sides):
        if not frame or not frame.get('session'):
            raise ValueError('Connect live PICO input before saving the matching pose')
        taught = {}
        for side in sides:
            identity = tracker_identity(config, side)
            taught[side] = dict(identity,
                               human=wrist_in_head_frame(frame, identity['serial'], identity['offset']),
                               robot=robot_wrist(robot_poses[side]))
        value = {'version': 1, 'id': uuid.uuid4().hex,
                 'saved_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()), 'sides': taught}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(value, allow_nan=False)+'\n')
        temporary.chmod(0o600)
        temporary.replace(self.path)
        return value

    def assess(self, frame, robot_poses, config, sides, now=None):
        result = {'calibrated': False, 'ready': False, 'sides': {}, 'robot_matches': False,
                  'position_tolerance_m': POSITION_M, 'orientation_tolerance_deg': ORIENTATION_DEG,
                  'hold_seconds': HOLD_SECONDS, 'evaluated_mono': time.monotonic() if now is None else now,
                  'reference_id': None, 'input_session': frame.get('session') if frame else None,
                  'frame_seq': frame.get('seq') if frame else None,
                  'message': 'Prepare arms, match the robot, then Save matching pose'}
        try:
            value = json.loads(self.path.read_text())
            if value.get('version') != 1 or not value.get('id'):
                raise ValueError('Saved matching pose is invalid; save it again')
            for side in sides:
                taught = value['sides'].get(side)
                identity = tracker_identity(config, side)
                if not taught or any(taught.get(key) != val for key, val in identity.items()):
                    raise ValueError('Tracker assignment or mounting offset changed; save the matching pose again')
                if not all(valid_reference_pose(taught.get(key)) for key in ('human', 'robot')):
                    raise ValueError('Saved matching pose is invalid; save it again')
            result.update(calibrated=True, reference_id=value['id'])
        except FileNotFoundError:
            return result
        except (OSError, ValueError, KeyError, TypeError, IndexError, AttributeError) as exc:
            result['message'] = str(exc) or 'Saved matching pose is invalid; save it again'
            return result
        if not frame:
            result['message'] = 'Waiting for live PICO tracking'
            return result
        try:
            robot_matches = True
            for side in sides:
                taught = value['sides'][side]
                human = wrist_in_head_frame(frame, taught['serial'], taught['offset'])
                robot = robot_wrist(robot_poses[side])
                pos = distance(human['position'], taught['human']['position'])
                angle = angle_degrees(human['rotation'], taught['human']['rotation'])
                robot_ok = (distance(robot['position'], taught['robot']['position']) <= ROBOT_POSITION_M and
                            angle_degrees(robot['rotation'], taught['robot']['rotation']) <= ROBOT_ORIENTATION_DEG)
                robot_matches = robot_matches and robot_ok
                result['sides'][side] = {
                    'position_error_m': round(pos, 4), 'orientation_error_deg': round(angle, 1),
                    'move_m': [round(taught['human']['position'][i]-human['position'][i], 4) for i in range(3)],
                    'matched': pos <= POSITION_M and angle <= ORIENTATION_DEG,
                    'robot_matches': robot_ok}
            result['robot_matches'] = robot_matches
            result['ready'] = robot_matches and all(v['matched'] for v in result['sides'].values())
            result['message'] = ('Robot ready pose changed; prepare arms and save the matching pose again' if not robot_matches else
                                 'Both wrists match; hold still' if result['ready'] and len(sides) == 2 else
                                 'Left wrist matches; hold still' if result['ready'] else
                                 'Match wrist positions and angles to the saved robot-ready pose; look forward')
        except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
            result.update(ready=False, message=str(exc) or 'Saved matching pose is invalid; save it again')
        return result


class MatchDwell:
    """Require a continuous match; stale data and reference/session changes reset it."""
    def __init__(self):
        self.since = None
        self.identity = None

    def update(self, assessment, input_ready, now):
        identity = (assessment.get('reference_id'), assessment.get('input_session'))
        age = now-assessment.get('evaluated_mono', float('-inf'))
        valid = (input_ready and assessment.get('ready') and all(identity) and
                 0 <= age <= ASSESSMENT_MAX_AGE)
        if not valid or identity != self.identity:
            self.since = None
        self.identity = identity
        if valid and self.since is None:
            self.since = now
        elapsed = 0. if self.since is None else max(0., now-self.since)
        return {'complete': valid and elapsed >= HOLD_SECONDS,
                'progress': min(1., elapsed/HOLD_SECONDS), 'hold_seconds': HOLD_SECONDS}
