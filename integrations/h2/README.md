# H2 PICO, Wuji and head-camera integration

This directory contains the H2 operator stack developed on a Unitree H2 with
Dex5 hands and a PICO 4 Ultra Enterprise. It adds a desktop dashboard, browser
Quest/PICO modes, native PICO wrist trackers with Wuji gloves, head following,
PC3 head video, recording and Wi-Fi/Ethernet selection.

It is isolated from this fork's normal `teleop/` entry points. The H2 runtime
is pinned to Unitree PR321 (`39b79bca34b7547b21edad3fe3e642bb5cac7147`) so the
same arm controller, IK, models and Dex5 interfaces can be reproduced.
No change to the running robot is made by cloning or bootstrapping this branch.

## Relationship to RoboticsCenter/main

RoboticsCenter/main at `d0de7e6` is the Unitree `817fb00` base plus two launcher
and setup commits. This integration uses the related PR321 branch for Dex5.
The two runtimes are not interchangeable: their H2 joint-index maps,
velocity/startup logic and one IK regularization weight differ. This branch
keeps the tested H2 arm controller and IK byte-identical to PR321, rather than
replacing RoboticsCenter's existing controller files.

Local adaptations are explicit in `sources.json` and `patches/`: Dex5 gain
settings, wrist-camera capture/composition, and the small browser video panel.
The native input, dashboard and video protocol adapters in this directory are
integration code, not a claim of a published, complete Unitree H2/PICO product.

## Prepare a checkout

From this directory, with Python3.10 or newer and Git installed:

```bash
python bootstrap.py
python bootstrap.py --check
```

Bootstrap initializes the pinned dependencies, checks and applies the listed
patches, copies configuration templates only when missing, and generates the
robot's linked left-hand model with mesh paths for this checkout. It preserves
existing operator settings. The resulting submodule worktrees show the
documented local patches; upstream commit pins remain unchanged.

Install the Unitree SDK and robot-side dependencies following the pinned
runtime's README. The tested robot environment used Python3.10, NumPy1.26.4,
Pinocchio/CasADi and Unitree SDK2. Laptop glove input used official
`wuji_sdk`2026.11.9. Native PICO input used XRoboToolkit PC Service1.0.0,
Python SDK1.1.0 and headset app1.1.1. Video encoding used a separate environment
with PyAV16.0.1 and NumPy1.26.4. The PC3 server uses the pinned official
TeleImager2.0.8 GStreamer backend and system GI bindings.

Generated settings under `config/` are ignored by Git. Set glove serials in
`config/wuji_gloves.json`, select wrist tracker serials on the dashboard, and
use the Wuji CLI to select/calibrate a local operator profile. Personal hand
models and calibration exports remain in the operator's private SDK storage.
The starter gain configuration uses the original1.00× gains; the dashboard
can save other values for the next launch.

The preserved deployment convention is `~/h2-wuji-teleop` on the laptop and
`/home/unitree/h2-upstream-pr321` on PC2. A fresh deployment may link those
paths to this directory. Do not overwrite an existing live deployment during
review. The laptop gateway accepts `H2_REMOTE_ROOT` and `H2_REMOTE_PYTHON` if
the robot's deployment or Python path differs. Runtime Python environments
and TLS certificates are installed separately and are not committed.

Canonical dashboard source is `deployment/robot/quest_h2_web.py`; its root
copy is the deployed entry point. The same applies to `quest_wuji.py`.
`deployment/robot/pc2_start_web.sh` starts the page without starting teleop.
`deployment/laptop_h2_socat.sh` supervises the laptop gateway/forwarders.
See [the operator flow](OPERATOR_FLOW.md) before launching motion.

The laptop gateway also serves `/wuji-calibration`, including when H2 is offline.
It provides named SDK profiles, glove discovery/assignment, the official live
hand-model calibration workflow, cancellation, backup and save verification.
See [glove calibration](deployment/WUJI_CALIBRATION.md). Calibration begins only
when the operator selects a glove/profile/hand and starts it on that page.

## Supported modes

- Browser Quest/PICO wrist and finger tracking.
- Browser arms with Wuji left fingers.
- Native PICO left wrist tracker with a Wuji left glove.
- Native PICO both wrist trackers with both Wuji gloves.
- Native PICO both wrist trackers with PICO optical finger tracking, without
  Wuji gloves (installed while teleop was idle; physical validation pending).
- Native PICO head-only following, holding the measured arm posture and
  leaving fingers idle.
- Right-finger-only testing and standalone right-hand grasp/hold.

The dashboard offers optional neck following for every browser and native control mode.
One upper-body writer combines the arm and head commands; it does not run
competing arm/head DDS publishers. Unitree's SDK supports FSM4 and703 for
these commands. Walking uses the existing Unitree motion controller/remote.
Leg-tracker locomotion and a dedicated arms-only mode are not implemented.

Native wrist modes now wait for a taught wrist pose before following. Prepare
arms, save a matching reference on first use, then Start pose check and hold the
match for one second. Arm/finger/head following and automatic recording begin
after the match. See [pose matching](deployment/PICO_POSE_MATCH.md).

## Camera

The confirmed PC3 is Jetson Thor / JetPack7.1 (R38.4), normally at
192.168.123.163. The S56 head camera is connected to PC3. Its supplied driver
package is `~/.Unitree/YUSHU_4A_AGTH_G2Y_7.1`; this branch does not redistribute
those proprietary drivers. Service templates are under `head_camera/systemd`.
The factory camera-power initialization stays in place. Argus uses the
documented `NVCAMERA_NITO_PATH=CONFIG` setting and its ISP file mode0644.

S56 → Argus/GStreamer → official TeleImager/ZMQ55558 → PC2 head JPEG endpoint
→ laptop video adapter → PICO Remote Vision. Native PICO selects ZEDMINI as
its compatible receiver option. The active source is the H2 **left eye**,
repeated for both display eyes; stereoscopic depth is not enabled.

Capture and desktop reception were verified at720p30. PICO viewing was
confirmed by the operator, typically15–20fps over the tested wireless path.
Both eyes captured individual1080p images; NVIDIA's stereo compositor attempt
failed, so it is not the active source. Recording now includes head.mp4 from the existing head-camera stream, plus
the existing wrist/joint captures. Automatic recording covers head-only sessions.
See [head following and video](deployment/HEAD_FOLLOW_VIDEO.md).

## Validation and remaining work

Separate arm/glove operation and a supported head-only test were exercised.
The new **PICO wrist trackers + PICO fingers** choice uses the official SDK hand
skeleton and pinned Unitree Dex5 controller. Its OpenXR conversion, independent
hand loss, START/STOP ownership and actual retargeting were checked offline.
See [deployment and operator steps](deployment/PICO_OPTICAL_HANDS.md).
The combined wrist/glove/head-camera attempt remained stopped because PICO
tracking/video connections were intermittent. The newest video recovery
change passed encode/decode and simulated source/socket interruption tests;
it still needs validation in a live combined session.

START currently treats the measured robot neck angle as neutral. A comfortable
headset pose can therefore inherit an existing downward neck angle. Recenter
also preserves the current robot angle. A saved forward-head calibration and
the requested arm neutral/reach calibration remain pending.

Tests are in `tests/`. They use fake command publishers or test transports;
running the suite does not start a physical teleoperation session:

```bash
python -m unittest discover -s tests
```

Run `test_head_video.py` in the video environment to exercise real H.264
encoding and network recovery. Other environments skip the codec-dependent
checks. Recordings, device logs, credentials, personal calibrations, generated
models and Python environments are excluded from this source branch.
