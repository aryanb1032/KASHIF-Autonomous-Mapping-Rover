# Verification report

Final checks completed 11 November 2026. No firmware was uploaded and no physical rover was driven.

## Results

| Check | Result |
|---|---|
| Python unit/integration tests | 24 passed, including real pseudo-terminal serial and keyboard tests |
| Native firmware parser and actual loop with fake hardware | Both compiled with warnings treated as errors and passed |
| ESP32 firmware | Compiled for original ESP32 Dev Module, Arduino core 2.0.17; 315,537 bytes flash and 22,536 bytes RAM |
| ROS packages | rover_description and rover_bringup built successfully; pinned official sllidar_ros2 also built |
| Gazebo navigation | All seven checked lifecycle nodes active; goal succeeded, 0.660 m displacement |
| Stop priority | Stop suppressed continuing nonzero commands |
| Operator connection loss | Expired heartbeat stopped motion despite continuing commands |
| Autonomous Gazebo mapping | Frontier goal reached; continued exploration; 0.423 m final displacement from initial pose |
| Map growth | Observed known cells increased from 1,500 to 21,159 |
| Exploration stop | Command gate disarmed, output zero, navigation canceled |
| RViz | Supplied configuration remained running for 8 seconds with OpenGL 4.5 |
| Shell syntax | Installation, flashing and test scripts passed bash syntax checks |

Raw results and logs are in `verification/`. The simulation tests exercise the launched ROS nodes, SLAM, Nav2, Gazebo physics and lidar. These are bounded smoke tests, not a guarantee of complete exploration or operation in every environment. The autonomous test measured movement and map growth; map-cell growth also includes sensor discovery while turning.

## Environment

Ubuntu 24.04 x86_64; ROS 2 Jazzy; Gazebo Harmonic 8.15; Nav2 1.3.13; slam_toolbox 2.8.5; robot_localization 3.8.3. Dependencies unavailable system-wide were extracted into an isolated local test prefix. This produces a benign colcon warning about that dependency prefix lacking local_setup files. The Pi installation script was syntax checked, but its sudo/apt installation was not executed on a Raspberry Pi. The tested Slamtec commit is recorded in README.md and setup_pi.sh.

## Remaining diagnostics

- KDL warns about inertia on the root base_link. Gazebo uses the inertia; robot_state_publisher still publishes transforms. No KDL dynamics are used here.
- Nav2 reports default behavior-tree error-code names and selects the sole configured goal/progress checker when the action does not name one.
- During the final exploration stop, Nav2 logged `Failed to get result for follow_path in node halt!`, then immediately reported goal cancellation; the controller reported successful cancellation about 60 ms later. The independent motion gate had already disarmed. This transient upstream cancellation diagnostic remains visible in the log; it was not suppressed.
- The upstream Slamtec driver produces compiler pedantic warnings about zero-length arrays, but builds successfully.

## Required physical commissioning

The hardware configuration deliberately ships with `hardware_calibrated: false`. Software is prepared for compilation and simulation; unrestricted physical operation cannot be verified from the component names alone.

1. Confirm the ESP32 is the original ESP32 variant supported by the supplied pins/target.
2. Resolve the originally described diagonal powered-wheel layout. The shipped model requires a common left/right axle; its simulation does not validate a diagonal drive arrangement.
3. Measure wheel radius, track spacing, encoder transitions per output-wheel turn, chassis footprint and sensor transforms. The shipped dimensions and 1,680 counts/revolution are examples.
4. Verify encoder logic levels, wiring, motor/encoder direction, motor voltage/current, independent E-stop and power supply.
5. Calibrate stationary gyro bias and tune motor speed gains with wheels raised, then test stopping and low-speed navigation in a clear bounded area.

Physical USB communication, real RPLIDAR/MPU9250 data, Pi ARM64 execution, wheel slip, motor PI tuning and battery behavior have not been tested. Hardware upload and commissioning remain to be done on the actual rover. See README.md for commands and wiring.
