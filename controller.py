#!/usr/bin/env python3
import json
import math
import socket
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

# Gazebo Classic services (we try both variants depending on your gazebo_msgs version)
SetEntityState = None
EntityState = None
SetModelState = None
ModelState = None

try:
    from gazebo_msgs.srv import SetEntityState  # type: ignore
    from gazebo_msgs.msg import EntityState  # type: ignore
except Exception:
    pass

try:
    from gazebo_msgs.srv import SetModelState  # type: ignore
    from gazebo_msgs.msg import ModelState  # type: ignore
except Exception:
    pass


def clamp_int(x: int, lo: int, hi: int) -> int:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    """Yaw-only quaternion."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))  # x,y,z,w


class CmdVelToESP32WithGazeboFollower(Node):
    """
    One node that:
      1) Subscribes to /cmd_vel
      2) Computes mecanum wheel speeds
      3) Sends JSON to ESP32 over TCP (or dry-run prints)
      4) Integrates cmd_vel open-loop into odom->base_link TF (+ optional /odom)
      5) Publishes /joint_states to spin wheels in RViz
      6) Optionally "teleports" the Gazebo model to match the same open-loop pose

    IMPORTANT:
      If you enable the Gazebo follower, you should DISABLE/remove any Gazebo plugin that also
      drives the robot from cmd_vel (e.g. gazebo_ros_planar_move), otherwise you'll get conflicts/jitter.
    """

    def __init__(self) -> None:
        super().__init__("cmd_vel_to_esp32_with_gazebo_follower")

        # ---------- Core cmd_vel ----------
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_timeout_s", 0.3)

        # ---------- Mecanum geometry ----------
        # X-roller layout (your case): "x"
        self.declare_parameter("mecanum_layout", "x")  # "x" or "o" (mirrored)
        self.declare_parameter("wheel_radius", 0.04)
        self.declare_parameter("lx", 0.073)   # half-length to wheel center (x)
        self.declare_parameter("ly", 0.0955)  # half-width  to wheel center (y)

        # Mapping/sign only affects what is sent to ESP32 (to match wiring).
        # It does NOT change the internal "physical wheel speed" calculation used for RViz wheel spin.
        self.declare_parameter("wheel_index_map", [0, 1, 2, 3])  # from [FL,FR,RL,RR] to output order
        self.declare_parameter("wheel_sign", [1, 1, 1, 1])       # sign after mapping

        # ---------- ESP32 TCP sending ----------
        self.declare_parameter("dry_run", True)  # if true: print JSON instead of TCP
        self.declare_parameter("esp32_ip", "172.30.101.210")
        self.declare_parameter("port", 5000)
        self.declare_parameter("tcp_timeout_s", 0.05)
        self.declare_parameter("reconnect_period_s", 1.0)

        self.declare_parameter("send_hz", 30.0)
        self.declare_parameter("send_mode", "pwm")  # "pwm" or "radps"
        self.declare_parameter("max_pwm", 255)
        self.declare_parameter("max_wheel_radps", 30.0)

        # ---------- Open-loop odom + TF (for RViz) ----------
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_joint_states", True)

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("base_z", 0.04)

        self.declare_parameter("odom_hz", 50.0)

        self.declare_parameter(
            "wheel_joint_names",
            [
                "front_left_wheel_joint",
                "front_right_wheel_joint",
                "rear_left_wheel_joint",
                "rear_right_wheel_joint",
            ],
        )

        # ---------- Gazebo follower ----------
        self.declare_parameter("gazebo_follow_enabled", True)
        self.declare_parameter("gazebo_model_name", "my_mecanum_robot")
        self.declare_parameter("gazebo_rate_hz", 30.0)

        # ---------- State ----------
        self.sock: Optional[socket.socket] = None
        self.last_connect_attempt = 0.0

        self.latest_cmd = Twist()
        self.last_cmd_wall_time = time.time()

        # Open-loop pose in odom frame
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Wheel positions (for /joint_states visualization), order: [FL, FR, RL, RR]
        self.wheel_pos = [0.0, 0.0, 0.0, 0.0]

        # ROS interfaces
        topic = str(self.get_parameter("cmd_vel_topic").value)
        self.sub = self.create_subscription(Twist, topic, self.cmd_vel_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, str(self.get_parameter("odom_topic").value), 10)
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)

        # Gazebo clients (created even if disabled; we simply won't call them)
        self.gz_entity_client = None
        self.gz_model_client = None
        if SetEntityState is not None:
            self.gz_entity_client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        if SetModelState is not None:
            self.gz_model_client = self.create_client(SetModelState, "/gazebo/set_model_state")

        # Timers
        send_hz = float(self.get_parameter("send_hz").value)
        send_period = 1.0 / send_hz if send_hz > 0.0 else 0.033
        self.send_timer = self.create_timer(send_period, self.on_send_timer)

        odom_hz = float(self.get_parameter("odom_hz").value)
        odom_period = 1.0 / odom_hz if odom_hz > 0.0 else 0.02
        self.last_odom_stamp = self.get_clock().now()
        self.odom_timer = self.create_timer(odom_period, self.on_odom_timer)

        gz_hz = float(self.get_parameter("gazebo_rate_hz").value)
        gz_period = 1.0 / gz_hz if gz_hz > 0.0 else 0.033
        self.gz_timer = self.create_timer(gz_period, self.on_gazebo_timer)

        self.get_logger().info(f"Subscribing to: {topic}")
        self.get_logger().info(f"ESP32 mode: {'DRY_RUN (print JSON)' if self.is_dry_run() else 'TCP send'}")

    def is_dry_run(self) -> bool:
        return bool(self.get_parameter("dry_run").value)

    # ---------- cmd_vel ----------
    def cmd_vel_callback(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.last_cmd_wall_time = time.time()

    def _cmd_is_timed_out(self) -> bool:
        timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        return (time.time() - self.last_cmd_wall_time) > timeout_s

    def _get_cmd(self) -> Tuple[float, float, float]:
        if self._cmd_is_timed_out():
            return 0.0, 0.0, 0.0
        return float(self.latest_cmd.linear.x), float(self.latest_cmd.linear.y), float(self.latest_cmd.angular.z)

    # ---------- Kinematics ----------
    def mecanum_inverse_kinematics(self, vx: float, vy: float, wz: float) -> List[float]:
        """
        Returns wheel angular speeds [rad/s] in order [FL, FR, RL, RR].
        Layout "x" is your X-roller pattern.
        """
        r = float(self.get_parameter("wheel_radius").value)
        lx = float(self.get_parameter("lx").value)
        ly = float(self.get_parameter("ly").value)
        if r <= 0.0:
            r = 1.0
        a = lx + ly

        layout = str(self.get_parameter("mecanum_layout").value).strip().lower()
        if layout == "o":
            w_fl = (vx + vy - a * wz) / r
            w_fr = (vx - vy + a * wz) / r
            w_rl = (vx - vy - a * wz) / r
            w_rr = (vx + vy + a * wz) / r
        else:
            # X-rollers
            w_fl = (vx - vy - a * wz) / r
            w_fr = (vx + vy + a * wz) / r
            w_rl = (vx + vy - a * wz) / r
            w_rr = (vx - vy + a * wz) / r

        return [w_fl, w_fr, w_rl, w_rr]

    def apply_output_mapping(self, wheels_flfrlrr: List[float]) -> List[float]:
        """Applies wheel_index_map and wheel_sign for ESP32 output only."""
        idx_map = list(self.get_parameter("wheel_index_map").value)
        sign = list(self.get_parameter("wheel_sign").value)

        if len(idx_map) != 4:
            idx_map = [0, 1, 2, 3]
        if len(sign) != 4:
            sign = [1, 1, 1, 1]

        out = [0.0, 0.0, 0.0, 0.0]
        for i in range(4):
            j = int(idx_map[i])
            if j < 0 or j > 3:
                j = i
            s = int(sign[i])
            if s not in (-1, 1):
                s = 1
            out[i] = float(wheels_flfrlrr[j]) * float(s)
        return out

    def scale_wheels_preserve_ratios(self, wheels: List[float], max_abs: float) -> List[float]:
        if max_abs <= 0.0:
            return wheels
        m = max(abs(w) for w in wheels) if wheels else 0.0
        if m <= max_abs or m < 1e-12:
            return wheels
        scale = max_abs / m
        return [w * scale for w in wheels]

    def wheels_to_pwm(self, wheels_radps: List[float]) -> List[int]:
        max_pwm = int(self.get_parameter("max_pwm").value)
        max_w = float(self.get_parameter("max_wheel_radps").value)
        if max_pwm <= 0:
            max_pwm = 255
        if max_w <= 0.0:
            max_w = 1.0

        wheels_radps = self.scale_wheels_preserve_ratios(wheels_radps, max_w)

        out: List[int] = []
        for w in wheels_radps:
            pwm = int(round((w / max_w) * max_pwm))
            out.append(clamp_int(pwm, -max_pwm, max_pwm))
        return out

    # ---------- ESP32 TCP ----------
    def _close_socket(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None

    def _connect_if_needed(self) -> None:
        if self.is_dry_run():
            return
        if self.sock is not None:
            return

        now = time.time()
        reconnect_period = float(self.get_parameter("reconnect_period_s").value)
        if (now - self.last_connect_attempt) < reconnect_period:
            return
        self.last_connect_attempt = now

        ip = str(self.get_parameter("esp32_ip").value)
        port = int(self.get_parameter("port").value)
        tcp_timeout_s = float(self.get_parameter("tcp_timeout_s").value)

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(tcp_timeout_s)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.connect((ip, port))
            self.sock = s
            self.get_logger().info(f"Connected to ESP32 at {ip}:{port}")
        except Exception as e:
            self._close_socket()
            self.get_logger().warn(f"ESP32 connection failed: {e}")

    def _send_line(self, line: str) -> None:
        if self.is_dry_run():
            # Print exactly what would be sent
            print(line, flush=True)
            return
        if self.sock is None:
            return
        try:
            self.sock.sendall((line + "\n").encode("utf-8"))
        except Exception as e:
            self.get_logger().warn(f"Send failed (dropping connection): {e}")
            self._close_socket()

    # ---------- Timers ----------
    def on_send_timer(self) -> None:
        self._connect_if_needed()

        vx, vy, wz = self._get_cmd()
        physical_wheels = self.mecanum_inverse_kinematics(vx, vy, wz)
        wheels_out = self.apply_output_mapping(physical_wheels)

        mode = str(self.get_parameter("send_mode").value).strip().lower()
        # Accept common typo/alias
        if mode in ("rads", "rad/s", "radps", "rad_per_s", "radpers"):
            mode = "radps"
        else:
            mode = "pwm"

        now_wall = time.time()
        if mode == "radps":
            max_w = float(self.get_parameter("max_wheel_radps").value)
            wheels_send = self.scale_wheels_preserve_ratios(wheels_out, max_w) if max_w > 0.0 else wheels_out
            payload = {"u": "radps", "w": wheels_send, "t": now_wall}
        else:
            payload = {"u": "pwm", "w": self.wheels_to_pwm(wheels_out), "t": now_wall}

        self._send_line(json.dumps(payload))

    def on_odom_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_odom_stamp).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_odom_stamp = now

        vx, vy, wz = self._get_cmd()

        # Integrate cmd_vel given in base frame into odom frame
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        vx_w = vx * c - vy * s
        vy_w = vx * s + vy * c

        self.x += vx_w * dt
        self.y += vy_w * dt
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))

        frame_id = str(self.get_parameter("frame_id").value)
        child_id = str(self.get_parameter("child_frame_id").value)
        base_z = float(self.get_parameter("base_z").value)
        qx, qy, qz, qw = yaw_to_quat(self.yaw)

        if bool(self.get_parameter("publish_tf").value):
            tfm = TransformStamped()
            tfm.header.stamp = now.to_msg()
            tfm.header.frame_id = frame_id
            tfm.child_frame_id = child_id
            tfm.transform.translation.x = self.x
            tfm.transform.translation.y = self.y
            tfm.transform.translation.z = base_z
            tfm.transform.rotation.x = qx
            tfm.transform.rotation.y = qy
            tfm.transform.rotation.z = qz
            tfm.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tfm)

        if bool(self.get_parameter("publish_odom").value):
            od = Odometry()
            od.header.stamp = now.to_msg()
            od.header.frame_id = frame_id
            od.child_frame_id = child_id
            od.pose.pose.position.x = self.x
            od.pose.pose.position.y = self.y
            od.pose.pose.position.z = base_z
            od.pose.pose.orientation.x = qx
            od.pose.pose.orientation.y = qy
            od.pose.pose.orientation.z = qz
            od.pose.pose.orientation.w = qw
            od.twist.twist.linear.x = vx
            od.twist.twist.linear.y = vy
            od.twist.twist.angular.z = wz
            self.odom_pub.publish(od)

        if bool(self.get_parameter("publish_joint_states").value):
            # Use PHYSICAL wheel speeds (FL,FR,RL,RR) for visualization
            wheels = self.mecanum_inverse_kinematics(vx, vy, wz)
            for i in range(4):
                self.wheel_pos[i] += wheels[i] * dt

            js = JointState()
            js.header.stamp = now.to_msg()
            js.name = list(self.get_parameter("wheel_joint_names").value)
            js.position = list(self.wheel_pos)
            js.velocity = list(wheels)
            self.js_pub.publish(js)

    def on_gazebo_timer(self) -> None:
        if not bool(self.get_parameter("gazebo_follow_enabled").value):
            return

        # Build pose from our open-loop state
        base_z = float(self.get_parameter("base_z").value)
        qx, qy, qz, qw = yaw_to_quat(self.yaw)

        model_name = str(self.get_parameter("gazebo_model_name").value)

        # We optionally provide twist too (in base frame it’s cmd_vel; in world it depends on yaw).
        # Gazebo teleporting works fine even if twist is left zero; pose is what matters.
        vx, vy, wz = self._get_cmd()
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        vx_w = vx * c - vy * s
        vy_w = vx * s + vy * c

        # Prefer SetEntityState if available
        if self.gz_entity_client is not None and self.gz_entity_client.service_is_ready():
            req = SetEntityState.Request()
            state = EntityState()
            state.name = model_name
            state.pose.position.x = self.x
            state.pose.position.y = self.y
            state.pose.position.z = base_z
            state.pose.orientation.x = qx
            state.pose.orientation.y = qy
            state.pose.orientation.z = qz
            state.pose.orientation.w = qw
            state.twist.linear.x = vx_w
            state.twist.linear.y = vy_w
            state.twist.angular.z = wz
            if hasattr(state, "reference_frame"):
                state.reference_frame = "world"
            req.state = state
            self.gz_entity_client.call_async(req)
            return

        # Fallback SetModelState
        if self.gz_model_client is not None and self.gz_model_client.service_is_ready():
            req = SetModelState.Request()
            state = ModelState()
            state.model_name = model_name
            state.pose.position.x = self.x
            state.pose.position.y = self.y
            state.pose.position.z = base_z
            state.pose.orientation.x = qx
            state.pose.orientation.y = qy
            state.pose.orientation.z = qz
            state.pose.orientation.w = qw
            state.twist.linear.x = vx_w
            state.twist.linear.y = vy_w
            state.twist.angular.z = wz
            state.reference_frame = "world"
            req.model_state = state
            self.gz_model_client.call_async(req)

    def destroy_node(self) -> bool:
        self._close_socket()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToESP32WithGazeboFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
