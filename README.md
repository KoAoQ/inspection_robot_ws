# Inspection Robot V3 Candidate Baseline

This workspace is a candidate formal baseline for the independent inspection robot. It does not depend on the original robot workspace at runtime.

## Frozen architecture target

`manual/nav/follow -> inspection_robot_core -> /control/cmd_vel -> inspection_robot_safety -> /base/safe_cmd_vel -> inspection_robot_base -> STM32`

The base driver owns the hardware serial port exclusively. ROS callbacks never write the serial port; the `BaseIoWorker` thread is the only writer and reader.

## Important commissioning warning

Do **not** run on the floor yet. First build on ROS 2 Humble, then bench-test with wheels lifted. The MCU safety `0xB1` command and the legacy motion-frame reserved byte are kept separate intentionally and must be confirmed against the real STM32 firmware. Ultrasonic TF positions in the simplified URDF are provisional and must be measured before using the 0.15 m hard-stop as a certified physical distance.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/inspection_robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

## First bench bringup

```bash
ros2 launch inspection_robot_bringup robot.launch.py enable_localization:=false
```

The velocity manager starts in `STOP`. Set `MANUAL` only after `/base/status` is healthy and all ultrasonic A-D inputs are verified.
