# Dashboard glove calibration

The laptop gateway serves `/wuji-calibration` directly, independently of H2.
The main teleop page links to it under Inputs. If H2 is unavailable, the laptop
dashboard's offline page also links to calibration.

## Operator flow

1. Power/connect the glove and open the calibration page on the laptop.
2. Select and activate the intended named Wuji SDK profile, or create one.
   Each person has a left and a right hand model. Default cannot store them.
3. Choose the hand and glove serial. Scan again if the device is not listed.
4. To reuse an existing fit, choose **Use selected glove with saved calibration**.
   Otherwise wear the glove, stop teleop/hand tests, and check the readiness box.
5. Press **Calibrate left/right glove**. If the profile already has that hand's
   model, confirm replacement. A private calibration bundle is exported first.
6. Follow the live text instructions: thumb to index, middle, ring and pinky;
   bend the four fingers about 90 degrees; then hold a flat open hand. The CLI
   detects the poses and advances automatically. Follow its displayed order.
7. Wait for the fit/publish result. Cancel sends one interrupt and waits for
   the CLI to release the glove. Failure/cancellation never starts a retry.
8. After success, return to teleop, select a Wuji mode and reconnect glove input.
   Calibrate the other hand separately if needed.

Use **Hand model** calibration for glove finger tracking. The tactile/contact
model workflow is separate and is not part of this page. Neither operation is
a PICO arm/reach calibration.

## Processes and stored data

`wuji_glove/calibration.py` manages the official `wuji` CLI. Profile and device
queries use structured JSON. A single `wuji calib hand-model --sn … --jsonl`
process provides live feedback and a terminal event. The page checks the
reported hand side, bound/result profile, serial, terminal event and exit code.
No OS users or SDK calibration registry files are edited by this integration.

The gateway closes its glove input bridges before fitting or switching profiles,
and rejects calibration while an online teleop/hand test is active. Glove
connection and teleop START/Prepare wait until an active calibration finishes.
No robot motion command is issued by the calibration manager.

Existing models are replaced only after the selected profile/hand overwrite
confirmation. The official CLI owns saving and loading the hand model. Backups
and calibration transcripts stay under
`~/.local/state/h2-teleop/wuji-calibration/`, outside this source checkout.
Glove serial assignments stay in the ignored local `config/wuji_gloves.json`.
No calibration, profile creation/switching or model replacement runs on page load.

## Deployment

Laptop files: `wuji_glove/dashboard_gateway.py`, `wuji_glove/calibration.py` and
`wuji_glove/calibration.html`. Restart the existing laptop gateway after updating
them. It serves calibration even while the robot is powered off.

The robot dashboard link is in `deployment/robot/quest_h2_web.py`; deploy its
root copy on H2 when it is available. Calibration itself needs no robot-side
calibration process or SDK installation.

Tests exercise the job lifecycle with a simulated CLI and browser HTTP routes.
The real installed CLI was queried for profiles, help and device discovery while
adding this feature; existing operator models were preserved. The laptop page
and robot dashboard link are installed. A live pose-fitting session
starts only when the operator presses the calibration button.
