# H2 walking modes

The dashboard distinguishes two Unitree body modes:

| Control | Mode | Arm teleoperation |
| --- | --- | --- |
| Enable Walk/Run | 601, matching the remote's Walk/Run selection | Not supported by Unitree's published arm SDK example |
| Arm-compatible walking (703) | 703 | Available after both mode and Arm SDK status are confirmed |

The Walk/Run button uses the official `LocoClient.SetFsmId(601)` request,
which is also what `LocoClient.Start()` sends. It does not enable Arm SDK or
send walking velocities. Use the Unitree remote for walking.

Set the body mode before preparing arm teleop. A body-mode request is rejected
while arm teleop is running. Mode 601 does not enable Prepare arms: Unitree's
published arm example accepts only 4 and 703. An enabled Arm SDK flag alone
does not establish compatibility with the current mode.

From fixed stand (4), the combined arm-compatible walking control first
requests and confirms Walk/Run (601), then requests and confirms 703, and
finally enables and verifies Arm SDK. If already in 601 it requests only 703;
if already in 703 it verifies and enables Arm SDK without changing body mode.

Each mode request waits for reported state. If the requested state does not
appear within ten seconds, the dashboard reports the last observed mode and
does not retry or select another mode. Server startup and page reload send no
body-mode commands. Dashboard Stop following ends teleop; it does not stop
walking or change the selected body mode.

References:

- [Official H2 locomotion client](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/h2/loco/h2_loco_client.py)
- [Official H2 arm SDK example](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/h2/high_level/h2_arm_sdk_dds_example.py)

Observed on the connected H2: direct requests from 4 to 703 were acknowledged
but remained in 4. After the operator selected Walk/Run (601), the dashboard's
601-to-703 request succeeded. This observation motivates the intermediate
Walk/Run stage; the firmware's internal reason for rejecting the direct
transition is not established.
