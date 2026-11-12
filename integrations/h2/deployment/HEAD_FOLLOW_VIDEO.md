# Head following and head-camera video

**Follow headset** is available in every dashboard control mode: browser
PICO/Quest, browser arms with a Wuji left glove, native PICO wrist trackers
with either optical fingers or Wuji gloves, and native head-only mode.

## Start a session

1. Select the arm/hand mode and enable **Follow headset** before Prepare.
2. Native PICO uses XRoboToolkit **Head + Send**, plus that mode's wrist and
   finger inputs. Browser modes use the headset browser's active XR view.
3. Prepare as usual. Look comfortably forward at START. Native wrist modes
   complete their existing saved-pose match first; then head following begins.
4. **Recenter** captures a new headset reference at the current robot neck angle.
   **Stop following** ends head following together with that session.

Native head-only mode always follows the headset, holds the measured arm
posture, and leaves fingers idle. Other modes use the existing arm/hand flow.
START still treats the current robot neck angle as neutral. This addition
does not automatically level a previously downward robot neck.

## Video recording

The existing manual REC button and automatic session recorder now include
**head.mp4**, a playable H.264 MP4 from the live PC3 head camera. Automatic
recording starts when following actually begins, including head-only mode,
and saves when the teleop process exits. STOP REC pauses automatic recording
for that session. Existing wrist recordings and joint logs remain available;
no new head-tracking or neck-motion dataset is added.

Files are under `/home/unitree/h2-recordings/<take>/` on PC2:

- `head.mp4`: the head camera's left-eye view.
- `head_frames.csv`: frame timestamps on the PC2 clock.
- `head_encoder.log`: codec errors, if any.
- `meta.json` / `status.json`: source, frame counts and recording status.

This is the same left-eye feed used by the desktop preview and PICO POV;
stereoscopic capture is not enabled. The recorder subscribes to the existing
PC3 TeleImager publisher, without opening the physical camera again. A source
loss is reported on the dashboard; the rest of the recording can continue.

The recorder requires system FFmpeg with libx264 support. It runs separately
from teleop with two encoding threads and uses fragmented MP4, retaining
completed fragments if power is lost. Normal STOP REC finalizes the file.

## Implementation and deployment

Native head mapping uses the existing `HeadPoseMapper` and `HeadCommandPublisher`.
Browser modes run the pinned official Unitree entry point through
`head_camera/browser_teleop.py`. Its integration hooks observe the official
CAMERA_MOVE handler, track its freshness through shared memory, preserve arm
and hand observations, and insert the neck target into the existing H2 writer
before CRC calculation. No second upper-body DDS publisher is started. The
pinned Linux/Python3.10 runtime uses the existing fork-based Vuer process.

Robot files: `quest_h2_teleop.py`, `quest_h2_web.py`, `quest_wuji.py`,
`quest_auto_record.py`, `h2_record.py`, `head_camera/browser_head.py`,
`head_camera/browser_teleop.py`, `head_camera/recording.py`. The canonical web
and Wuji sources are in `deployment/robot/`; deploy their root entry-point copies.
Update only while teleop and recording are stopped. Preserve camera receiver
settings, wrist references, gloves, gains, credentials and firmware.

Validation covers all-mode selection, head readiness before glove activation,
browser freshness and recenter/STOP, shared-process camera events, preservation
of the upstream arm/hand observation, and actual MP4 decoding on PC2. Physical
head following through the new browser adapter still needs an operator test.
