# PICO controllers + Wuji gloves

Select **PICO controllers + both Wuji gloves** on the desktop dashboard.
Both controller positions and orientations drive the existing H2 arm/wrist
mapper; the two calibrated Wuji gloves control fingers. Physical wrist trackers
and PICO optical hand tracking are not required for this option.

1. Turn on both controllers and wear the calibrated gloves.
2. In native XRoboToolkit connect PC Service to the laptop's current Wi-Fi IP.
   Enable **Head**, **Controller**, and **Send**. Left/right roles are automatic.
3. Select the new dashboard mode, **Connect PICO controller input**, and
   **Connect both Wuji gloves**. Move both controllers to verify live poses.
4. Use the existing body-mode controls, **Prepare arms**, then look forward
   and press **Start following** in the intended neutral arm pose. The existing
   optional pose-match setting still applies; a controller reference is saved
   separately from the physical wrist-tracker reference if matching is enabled.
5. **Stop following** uses the existing arm, glove, and head-following STOP path.

The existing camera, optional Follow headset, recording, and saved finger/thumb
stiffness controls apply. Controller triggers/grips do not replace Wuji finger
input. Holding a controller occupies the fingers; physical wrist trackers remain
the option for leaving both hands free. Controller mounting offsets are not
automatically measured; START retains the existing relative pose mapping.

The input reader uses the official SDK's `get_left_controller_pose()` and
`get_right_controller_pose()` APIs, documented in
[XRoboToolkit PC Service Pybind](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind#using-the-python-bindings).
The native app's Head/Controller/Send controls are documented in the
[official Unity client](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client#unity-ui-main-panel-reference).
The official Python binding exposes no per-controller tracking-valid flag;
the dashboard reports valid poses and their pose-change age using the existing
wrist-input freshness check. Move each controller before START. There is no
fallback from controllers to optical hands or physical wrist trackers.

All input modes share one official SDK reader. Packet timestamps drive sampling
so controller/head input continues when a Motion Tracker timestamp stops.
The Unitree PR321 arm controller, IK, hand retargeter and existing gains are
unchanged. Physical following must be checked with the operator after deployment.
