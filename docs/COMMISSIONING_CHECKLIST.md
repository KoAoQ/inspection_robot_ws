# Commissioning checklist

1. Confirm `/dev/inspection_robot_controller` udev symlink.
2. `colcon build` and `colcon test` on ROS 2 Humble.
3. Wheels lifted; no person/object near drivetrain.
4. Verify serial opens and `/base/status` enters WAITING_FOR_FEEDBACK then READY only after a new 24-byte frame.
5. Verify unplug/replug clears freshness and never resumes an old command.
6. Verify NaN/Inf test command latches fault.
7. Verify command timeout continuously sends zero.
8. Verify feedback freeze closes/reconnects serial and remains latched until reset.
9. Verify MCU safety command `0xB1` behavior against real firmware before floor operation.
10. Measure and correct ultrasonic A-F URDF transforms. Confirm invalid/stale A-D causes fail-closed stop.
11. Confirm 0.15 m threshold against actual bumper clearance.
12. Only then enable localization and later Nav2.
