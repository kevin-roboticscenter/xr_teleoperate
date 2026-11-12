# Match the prepared robot before native PICO following

Native wrist modes (`pico_wuji`, `pico_dual_wuji`, `pico_tracker_hands`) use a
taught start-pose reference. The existing Unitree PR321 arm controller, IK,
Dex5 retargeter, gains and body-mode requests are unchanged.

Pose matching is optional. Set `config/pico_pose_match_settings.json` to
`{"enabled": false}` to use direct **Start following**, preserving the saved
reference. Set it to true to restore the check. Changing this setting cancels a
pending check without automatically starting following. Use Stop before updating
an already prepared process. Live wrist/head/finger input is still required.

## First use or changed wearer / tracker mounting

1. Connect the existing native PICO input and required finger source. In
   XRoboToolkit enable Head, Object Tracking, Send, and Hand for optical fingers.
2. Select a supported body mode and press **Prepare arms**. This performs the
   existing ready-position movement and opens the hands in optical-finger mode.
3. Physically match H2's prepared wrist positions and wrist angles. Look
   straight forward and press **Save matching pose** in the Start pose panel.
   This teaches the operator reference; it cannot verify the first reference
   without the person's visual alignment. Wuji finger calibration is separate.
4. Press **Start pose check**. Following remains idle while the dashboard shows
   left/right position and angle errors and directional position guidance.
5. Match the saved pose continuously for **1 second**, within **5 cm** and
   **15 degrees** for every selected wrist. Following starts automatically.

## Later sessions

Prepare arms, press Start pose check, then match the taught pose. The reference
survives reboots. It uses head-relative position and yaw-aligned orientation,
so changing room origin or facing direction alone does not invalidate it.
Look forward during matching. A changed tracker assignment or measured mounting
offset requires saving the reference again. A reference taught for both wrists
can be used left-only; a left-only reference does not authorize both wrists.

The actual robot wrist feedback must also remain within 2.5 cm / 10 degrees of
the taught robot-ready pose. If it differs, prepare again and check the reference.
This verifies wrists. Elbow configuration remains inferred by Unitree's IK;
it does not measure the operator's elbows or fit a body/reach model.

## Waiting, START and STOP

An out-of-tolerance wrist, missing required optical hand, stale input/heartbeat,
changed input session, or changed reference resets the continuous-match timer.
During the wait the prepared arm target holds. Wuji publishers, optical finger
following, optional head following, the follow timer and automatic recording
begin only after the match. The arm process rechecks live pose/robot feedback
when it receives the final START command.

**Stop following** cancels a pending match and performs the existing arm-home
and shutdown flow. A cancelled wait cannot start later. Tracking loss after
following has begun retains the existing last-target behavior; reconnecting
that input still requires Stop and a new Prepare/Start sequence.

Browser Quest/PICO and native head-only sessions retain their existing flow.
The wait is implemented in the PC2 dashboard; keep that server running.

## Files and deployment

- `pico_tracker/pose_match.py`: pure reference geometry and continuous-match timer.
- `pico_tracker/arm_teleop.py`: measured robot feedback, reference teaching and
  final START recheck using the existing controller.
- `deployment/robot/quest_h2_web.py`: Start wait, deferred finger activation,
  calibration endpoint and dashboard guidance. Deploy as root `quest_h2_web.py`.
- `config/pico_pose_match.json`: private runtime reference, ignored by Git.

Deploy these three Python files to the existing PC2 integration while arm/hand
tests are stopped, back up the replaced files, then restart only its dashboard
with the existing `pc2_start_web.sh`. Preserve all robot/operator configuration.
The laptop reader/gateway does not need an update or another SDK subscriber.

Offline verification covers room-frame invariance, wrong position/orientation,
robot-ready changes, reference invalidation, stale input, deferred publishers
and recording, cancellation and a real background wait with mocked controllers.
The first physical alignment/reference still needs the wearer at the robot.
