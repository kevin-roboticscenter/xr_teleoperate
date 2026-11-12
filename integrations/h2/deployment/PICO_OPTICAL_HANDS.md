# PICO wrist trackers + PICO optical fingers

This addition was prepared and tested offline, then installed on the existing
PC2 integration while teleop was idle. Physical following validation is pending.

## Operator flow after deployment

1. Open native XRoboToolkit in PICO. Set PC Service to the laptop's current
   address; enable Head, Hand, Object Tracking and Send.
2. On the desktop dashboard select **PICO wrist trackers + PICO fingers**.
3. Press **Connect PICO tracker input**, assign left/right wrist tracker serials,
   and show both hands until both optical finger inputs report tracked.
4. Use the existing supported H2 mode (4 or 703), then **Prepare arms**. The
   existing Unitree controller opens both hands during preparation.
5. Match the prepared arm pose, look forward, and press **Start following**.
6. **Stop following** ends finger publishing and returns the arms home through
   the existing controller. Walking stays under the Unitree remote.

Wuji gloves and their calibration profiles are not required for this mode.
Wrist trackers are the sole arm/wrist pose source. Optical hand skeletons
control fingers only. Existing finger/thumb gains still apply. If one optical
hand becomes unavailable, its last finger target holds while the other side
can keep following. Head following remains optional.

## Deployment file mapping

Copy the following laptop sources to the existing H2 integration root
`/home/unitree/h2-upstream-pr321` while teleop is stopped:

| Laptop source | Robot path relative to integration root |
|---|---|
| `quest_h2_teleop.py` | `quest_h2_teleop.py` |
| `deployment/robot/quest_h2_web.py` | `quest_h2_web.py` |
| `pico_tracker/hands.py` | `pico_tracker/hands.py` |
| `pico_tracker/input.py` | `pico_tracker/input.py` |
| `pico_tracker/stream_trackers.py` | `pico_tracker/stream_trackers.py` |
| `pico_tracker/arm_teleop.py` | `pico_tracker/arm_teleop.py` |

The SDK reader `stream_trackers.py` also runs on the laptop. The laptop gateway
source is `wuji_glove/dashboard_gateway.py`; restart the existing gateway after
deploying the robot files. Reconnect PICO input so its reader loads the new
hand-skeleton support. Restart the robot dashboard using its existing launch
procedure. These restarts do not select a body mode or start teleoperation.

Preserve `config/`, TLS certificates and all upstream controllers. The hand
landmark conversion follows the pinned Unitree `tv_wrapper.py`; finger commands
come from its existing `Dex5_1_Controller`. No upstream source was edited.

## Validation

Offline checks cover OpenXR palm removal and fingertip indices, coordinate-frame
invariance, per-hand input loss, input-session changes, controller cleanup,
Wuji-free START, and retaining tracker input when changing dashboard modes.
Actual Unitree hand retargeting was exercised for both hands at two model
postures with no DDS initialization or motor writes.

This feature does not implement the pending wrist-offset/reach calibration.
