#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
INC="$ROOT/src/inspection_robot_base/include"
OUT="${TMPDIR:-/tmp}/inspection_robot_v3_pure"
mkdir -p "$OUT"
for f in serial_frame_router motion_protocol ultrasonic_protocol charging_protocol device_control_protocol odometry_integrator base_fault_manager control_state hardware_state; do
  g++ -std=c++17 -Wall -Wextra -Wpedantic -I"$INC" -c "$ROOT/src/inspection_robot_base/src/$f.cpp" -o "$OUT/$f.o"
done
echo "V3 non-ROS core syntax check passed"
