# Starting the H2 setup

Replace `LAPTOP_IP` below with the laptop’s current Wi-Fi address. Device serials are configured locally; the examples contain no operator identifiers.

Desktop dashboard: http://LAPTOP_IP:8787/h2-teleop

Browser PICO/Quest link:
https://LAPTOP_IP:8012/?ws=wss://LAPTOP_IP:8012

Native PICO XRoboToolkit uses PC Service IP: `LAPTOP_IP`. For its head-camera
view, select Remote Vision → ZEDMINI → Listen → `LAPTOP_IP`. This is the
compatible receiver option for our H2 source. No browser link is needed for
native PICO modes. The live view repeats the H2 left-eye image for both eyes.

For a motion test, first open Robot mode and select the supported operating
mode: Prepare standing / FSM4 for supported testing. Changing input modes
requires Stop following. Save stiffness changes before Prepare arms.
Preparing an arm mode moves the robot arms toward their initial pose.

## Wuji glove calibration

Open `http://LAPTOP_IP:8787/wuji-calibration` on the laptop. This page works with
H2 powered off. Select/create the operator profile, scan/select the glove and
choose left or right. Reuse the saved calibration or start the guided six-pose
hand-model fit. Existing models require replacement confirmation and are backed
up first. After success, return to teleop and reconnect Wuji input.

See [the full workflow](deployment/WUJI_CALIBRATION.md).

## Browser arms and headset hands

1. Select **PICO / Quest browser · arms + hands**.
2. Press **Prepare arms**, then wait for Ready.
3. In the headset browser, open the browser link above. Enable hand tracking
   and enter the Vuer XR view / Pass-through.
4. With tracking active, hold the starting arm pose and press **Start following**.
5. **Stop following** ends the session and returns the arms home. Reload the
   headset browser after the next Prepare arms if its connection was closed.

This mode uses browser hand/wrist tracking for the arms and fingers. Native
XRoboToolkit head following and its camera view are separate from this mode.

## Browser arms and Wuji left fingers

1. Select **Browser arms + Wuji left hand**.
2. Power/connect the calibrated left glove and press **Connect Wuji glove**.
3. Press **Prepare arms**, enter the headset browser XR view, wait for Ready,
   then press **Start following**.
4. Browser poses drive both arms; the Wuji glove drives left fingers. Right
   fingers retain their last pose. **Stop following** ends the session.

## Native wrist start-pose reference

On first use, after Prepare arms, physically match the robot’s wrist positions
and angles, look forward, and press **Save matching pose**. Repeat after changing
the wearer or tracker mounting. **Start pose check** waits for all selected wrists
to match continuously for one second before arms, fingers and optional head
following begin. Stop cancels the wait. See [pose matching](deployment/PICO_POSE_MATCH.md).

## Native PICO wrist trackers and optical fingers

1. Power both wrist trackers. In XRoboToolkit enable **Head**, **Hand**,
   **Object Tracking** and **Send**, using `LAPTOP_IP` for PC Service.
2. Select **PICO wrist trackers + PICO fingers** on the dashboard.
3. Press **Connect PICO tracker input**. Verify left/right tracker assignments
   and that both hands are optically tracked. Wuji gloves are not required.
4. In supported robot mode 4 or 703, press **Prepare arms**. The hands open
   during preparation. Save the reference on first use, then **Start pose check**
   and hold the matching wrist pose until following begins.
5. Trackers drive arms and wrists; PICO optical hand tracking drives fingers.
   Stop ends both and returns the arms home. Walking stays under the remote.

This mode is installed; see [deployment](deployment/PICO_OPTICAL_HANDS.md). Physical following validation is pending.

## Native PICO left wrist and Wuji left hand

1. Power the left wrist tracker and calibrated left glove.
2. In XRoboToolkit connect PC Service to `LAPTOP_IP`, enable **Head**,
   select **Object Tracking** for the motion trackers, and enable **Send**.
3. On the dashboard select **PICO left wrist tracker + Wuji left hand**.
4. Press **Connect PICO tracker input** and **Connect Wuji glove**. Confirm
   the saved left wrist serial is the tracker mounted on that wrist.
5. Optional: select **Follow headset** before preparing. Open Remote Vision
   for the head-camera view.
6. Press **Prepare arms**, wait for Ready and save the reference on first use.
   Press **Start pose check**, look forward and hold the matching wrist pose.
7. The left wrist tracker drives the left arm/wrist, and the glove drives
   left fingers. The right side holds its starting pose. Stop ends following
   and returns the arms home.

## Native PICO both wrists and both Wuji gloves

1. Power both trackers and both calibrated gloves. Mount trackers rigidly on
   the corresponding wrists/hands and keep them visible to the headset.
2. XRoboToolkit: PC Service `LAPTOP_IP`; **Head ON**, **Object Tracking**,
   **Send ON**. Confirm two motion trackers appear.
3. Dashboard: **PICO wrist trackers + both Wuji gloves**.
4. Press **Connect PICO tracker input** and **Connect both Wuji gloves**.
   Confirm left/right tracker assignments under Wrist tracker assignments.
   Saved serials are left LEFT_TRACKER_SN, right RIGHT_TRACKER_SN.
5. Select **Follow headset** if desired and open Remote Vision for camera POV.
6. Press **Prepare arms**. After Ready, save the reference on first use. Press
   **Start pose check**, look forward and match both wrists until following starts.
7. Each tracker controls its arm/wrist; each glove controls its fingers.
   Stop ends following and returns the arms home. Walking remains under
   the Unitree remote.

Wuji gloves replace headset finger tracking in these native modes. Arm targets
come from the wrist trackers, not optical hand landmarks.

## Camera and head only

1. XRoboToolkit: PC Service `LAPTOP_IP`; **Head ON**, **Send ON**.
2. Open Remote Vision as above. Camera-only viewing needs no Prepare/Start.
3. For neck motion select **PICO head only · camera + neck** on the dashboard
   and press **Connect PICO headset**.
4. In a supported SDK operating mode, press **Prepare head tracking**, look
   comfortably forward, then press **Start head following**.
5. Arms hold their measured START posture. Finger control stays off. Stop ends
   this test without an arm-home sequence. Gloves and wrist trackers are not
   required for this mode.

The saved forward-head calibration fix is still pending. START currently maps
your headset reference to the robot's current neck angle. Recenter preserves
the current robot angle, so it does not level an already-downward head.

## Other dashboard operations

- **Individual hand test** opens the right-finger-only test. Connect the right
  glove and use START right-hand test; main Stop ends the test.
- **Standalone grasp** runs the right-hand bag hold without a headset and
  continues through Wi-Fi loss while the robot stays powered. Use **Release
  right hand** to open it; main Stop is not its release command.
- To combine remote walking with upper-body teleop, select the existing
  **Enable walking with arm teleop** option while following is stopped and
  confirm FSM703 before preparing arms. Use the Unitree remote for legs/body.
  Main Stop ends upper-body following, not walking. Leg-tracker locomotion
  is not implemented.

## Wi-Fi changes

The laptop, headset and robot need a reachable common network. Join H2 to a
new network through Unitree Explore; `h2-wifi` does not join the robot to an SSID.

Returning to the currently saved RoboticsCenter_5G network:

1. Stop following, connect all devices, and wait for H2 startup.
2. Run `h2-link wifi` on the laptop. It resolves the saved H2 hostname and
   restores the route to PC2 if its DHCP gateway address changed.
3. Run `h2-wifi` once PC2 is reachable to print the current page URLs and
   update the browser certificate if required. SSH login may be requested.
4. If the laptop IP changed, reopen the new dashboard/browser links and enter
   the new laptop IP in native PICO PC Service and Remote Vision Listen.
5. Reconnect glove/tracker input from the dashboard and start the chosen mode.

A different SSID or subnet needs a one-time configuration update: the current
route helper explicitly checks the saved Wi-Fi profile and subnet. The
assistant can update the profile/routing and verify PC2/camera access. Keeping
an Ethernet connection for this first setup makes recovery straightforward.
With H2 connected by Ethernet, `h2-link ethernet` selects that transport;
`h2-wifi` can then update the headset-facing laptop address/certificate.
Switch back to `h2-link wifi` after the new wireless route is configured.

Glove calibration, tracker serial assignments, robot controller code and the
robot's internal PC2/PC3 addresses do not need changing for a Wi-Fi switch.
Existing browser tabs retain their old address. Links generated inside an
opened dashboard follow that dashboard's hostname, but arbitrary network
changes are not fully automatic yet. Guest/client-isolated networks may block
the headset-to-laptop connection.
