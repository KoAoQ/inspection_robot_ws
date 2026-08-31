#!/usr/bin/env python3
"""Low-speed closed-loop motion utility for calibration and acceptance tests."""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from inspection_robot_interfaces.msg import SafetyStatus
from inspection_robot_interfaces.srv import SetControlMode
from nav_msgs.msg import Odometry
from rclpy.node import Node


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_odometry(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class ClosedLoopSegment(Node):
    def __init__(self) -> None:
        super().__init__("closed_loop_segment")
        self.command_publisher = self.create_publisher(Twist, "/cmd_vel/manual", 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_odometry, 20)
        self.create_subscription(SafetyStatus, "/safety/status", self.on_safety, 20)
        self.mode_client = self.create_client(SetControlMode, "/control/set_mode")

        self.x = None
        self.y = None
        self.yaw = None
        self.unwrapped_yaw = None
        self.last_yaw = None
        self.last_odometry_time = None
        self.safety = None
        self.last_safety_time = None
        self.unhealthy_since = None

    def on_odometry(self, message: Odometry) -> None:
        yaw = yaw_from_odometry(message)
        if self.last_yaw is None:
            self.unwrapped_yaw = yaw
        else:
            self.unwrapped_yaw += normalize_angle(yaw - self.last_yaw)
        self.last_yaw = yaw
        self.yaw = yaw
        self.x = message.pose.pose.position.x
        self.y = message.pose.pose.position.y
        self.last_odometry_time = time.monotonic()

    def on_safety(self, message: SafetyStatus) -> None:
        self.safety = message
        self.last_safety_time = time.monotonic()
        if message.healthy:
            self.unhealthy_since = None
        elif self.unhealthy_since is None:
            self.unhealthy_since = self.last_safety_time

    def set_mode(self, mode: str, timeout: float = 5.0) -> bool:
        if not self.mode_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("/control/set_mode service is unavailable")
            return False
        request = SetControlMode.Request()
        request.mode = mode
        future = self.mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            self.get_logger().error(f"failed to set control mode to {mode}")
            return False
        response = future.result()
        if not response.success:
            self.get_logger().error(response.message)
            return False
        return True

    def publish_zero(self) -> None:
        message = Twist()
        for _ in range(10):
            self.command_publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)

    def wait_until_ready(self, timeout: float = 8.0) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.x is not None and self.safety is not None:
                if self.safety.healthy and not self.safety.emergency_stop:
                    return True
        self.get_logger().error("fresh odometry and healthy safety status were not received")
        return False

    def check_runtime_health(self) -> None:
        now = time.monotonic()
        if self.last_odometry_time is None or now - self.last_odometry_time > 0.3:
            raise RuntimeError("filtered odometry is stale")
        if self.last_safety_time is None or now - self.last_safety_time > 0.3:
            raise RuntimeError("safety status is stale")
        if self.safety.emergency_stop:
            raise RuntimeError("emergency stop is active")
        if self.safety.obstacle_stop:
            raise RuntimeError("safety obstacle stop is active")
        if self.unhealthy_since is not None and now - self.unhealthy_since > 0.5:
            raise RuntimeError(f"safety is unhealthy: {self.safety.reason}")

    def stop(self) -> None:
        self.publish_zero()
        self.set_mode("STOP")


def run_distance(node: ClosedLoopSegment, args: argparse.Namespace) -> dict:
    direction = 1.0 if args.distance >= 0.0 else -1.0
    target = abs(args.distance)
    start_x = node.x
    start_y = node.y
    start_yaw = node.yaw
    started = time.monotonic()
    last_report = 0.0

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        node.check_runtime_health()
        dx = node.x - start_x
        dy = node.y - start_y
        forward = direction * (dx * math.cos(start_yaw) + dy * math.sin(start_yaw))
        lateral = -dx * math.sin(start_yaw) + dy * math.cos(start_yaw)
        remaining = target - forward
        if remaining <= args.distance_tolerance:
            break
        if time.monotonic() - started > args.timeout:
            raise RuntimeError("distance motion timed out")

        if remaining > 0.20:
            speed = args.linear_speed
        else:
            speed = max(0.025, args.linear_speed * remaining / 0.20)
        command = Twist()
        command.linear.x = direction * min(speed, args.linear_speed)
        node.command_publisher.publish(command)

        if time.monotonic() - last_report >= 1.0:
            print(
                f"distance progress={forward:.3f} m remaining={max(remaining, 0.0):.3f} m "
                f"lateral={lateral:.3f} m",
                flush=True,
            )
            last_report = time.monotonic()
        time.sleep(0.03)

    return {"start_x": start_x, "start_y": start_y, "start_yaw": start_yaw}


def run_turn(node: ClosedLoopSegment, args: argparse.Namespace) -> dict:
    direction = 1.0 if args.angle_deg >= 0.0 else -1.0
    target = math.radians(abs(args.angle_deg))
    start_unwrapped_yaw = node.unwrapped_yaw
    started = time.monotonic()
    last_report = 0.0

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        node.check_runtime_health()
        turned = direction * (node.unwrapped_yaw - start_unwrapped_yaw)
        remaining = target - turned
        if remaining <= math.radians(args.angle_tolerance_deg):
            break
        if time.monotonic() - started > args.timeout:
            raise RuntimeError("turn motion timed out")

        if remaining > math.radians(20.0):
            speed = args.angular_speed
        else:
            speed = max(
                0.035,
                args.angular_speed * remaining / math.radians(20.0),
            )
        command = Twist()
        command.angular.z = direction * min(speed, args.angular_speed)
        node.command_publisher.publish(command)

        if time.monotonic() - last_report >= 1.0:
            print(
                f"angle progress={math.degrees(turned):.1f} deg "
                f"remaining={max(math.degrees(remaining), 0.0):.1f} deg",
                flush=True,
            )
            last_report = time.monotonic()
        time.sleep(0.03)

    return {"start_unwrapped_yaw": start_unwrapped_yaw, "direction": direction}


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move one low-speed segment using /odometry/filtered feedback."
    )
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--distance", type=float, help="signed distance in metres")
    target.add_argument("--angle-deg", type=float, help="signed turn angle in degrees")
    parser.add_argument("--linear-speed", type=float, default=0.10)
    parser.add_argument("--angular-speed", type=float, default=0.18)
    parser.add_argument("--distance-tolerance", type=float, default=0.005)
    parser.add_argument("--angle-tolerance-deg", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=120.0)
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if args.distance is not None and abs(args.distance) < 0.01:
        print("distance target must be at least 0.01 m", file=sys.stderr)
        return 2
    if args.angle_deg is not None and abs(args.angle_deg) < 1.0:
        print("angle target must be at least 1 degree", file=sys.stderr)
        return 2
    if args.linear_speed <= 0.0 or args.linear_speed > 0.20:
        print("linear speed must be in (0, 0.20] m/s", file=sys.stderr)
        return 2
    if args.angular_speed <= 0.0 or args.angular_speed > 0.40:
        print("angular speed must be in (0, 0.40] rad/s", file=sys.stderr)
        return 2

    rclpy.init()
    node = ClosedLoopSegment()
    result = None
    success = False
    try:
        if not node.wait_until_ready():
            return 1
        if not node.set_mode("MANUAL"):
            return 1
        if args.distance is not None:
            result = run_distance(node, args)
        else:
            result = run_turn(node, args)
        success = True
    except (KeyboardInterrupt, RuntimeError) as error:
        node.get_logger().error(str(error))
    finally:
        node.stop()
        settle_deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        if result is not None and args.distance is not None:
            dx = node.x - result["start_x"]
            dy = node.y - result["start_y"]
            forward = math.copysign(1.0, args.distance) * (
                dx * math.cos(result["start_yaw"]) + dy * math.sin(result["start_yaw"])
            )
            lateral = -dx * math.sin(result["start_yaw"]) + dy * math.cos(result["start_yaw"])
            print(f"RESULT distance={forward:.4f} m lateral={lateral:.4f} m", flush=True)
        elif result is not None:
            turned = result["direction"] * (
                node.unwrapped_yaw - result["start_unwrapped_yaw"]
            )
            print(f"RESULT angle={math.degrees(turned):.2f} deg", flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
