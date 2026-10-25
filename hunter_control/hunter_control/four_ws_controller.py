"""HUNTeR four-wheel-steering drive controller node.

Turns velocity commands into knuckle angle and wheel rate commands for the
ros2_control forward controllers, and integrates wheel odometry.

Subscriptions
  cmd_vel                (geometry_msgs/Twist or TwistStamped, see use_stamped_vel)
  ackermann_cmd          (ackermann_msgs/AckermannDriveStamped) front-equivalent angle + speed
  ~/steering_mode        (std_msgs/String) front|rear|counter|crab|adaptive|free
  joint_states           (sensor_msgs/JointState) measured knuckle angles and wheel rates
Publications
  steering_position_controller/commands  (std_msgs/Float64MultiArray) [FL, FR, RL, RR] rad
  wheel_velocity_controller/commands     (std_msgs/Float64MultiArray) [FL, FR, RL, RR] rad/s
  odom                   (nav_msgs/Odometry) and TF odom -> base_link (publish_tf)
  ~/realised_twist       (geometry_msgs/TwistStamped) command after mode/limit projection
  ~/active_mode          (std_msgs/String, latched)
"""

from __future__ import annotations

import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import TransformBroadcaster

from .kinematics import MODE_CRAB, MODES, WHEELS, FourWheelSteeringKinematics
from .vehicle_config import geometry_from_config, load_vehicle

try:
    from ackermann_msgs.msg import AckermannDriveStamped
except ImportError:  # pragma: no cover
    AckermannDriveStamped = None


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class FourWsController(Node):
    def __init__(self) -> None:
        super().__init__("four_ws_controller")
        p = self.declare_parameter
        p("vehicle_config_file", "")
        p("steering_mode", "counter")
        p("differential", "electronic")        # electronic | locked
        p("linkage", "")                        # override vehicle.yaml: trapezoid | ideal
        p("use_stamped_vel", True)
        p("cmd_timeout", 0.5)
        p("update_rate", 50.0)
        p("publish_tf", True)
        p("odom_frame", "odom")
        p("base_frame", "base_link")
        p("steer_joints", [f"{w}_steer_joint" for w in WHEELS])
        p("wheel_joints", [f"{w}_wheel_joint" for w in WHEELS])
        p("steer_error_slowdown_deg", 8.0)      # start slowing when knuckles lag this much
        p("steer_error_stop_deg", 25.0)         # stop when lag reaches this

        cfg = load_vehicle(self.get_parameter("vehicle_config_file").value or None)
        lim = cfg["limits"]
        p("max_speed", float(lim["max_speed"]))
        p("max_reverse_speed", float(lim["max_reverse_speed"]))
        p("max_accel", float(lim["max_accel"]))
        p("max_decel", float(lim["max_decel"]))
        p("max_yaw_rate", float(lim["max_yaw_rate"]))
        p("max_lateral_speed", float(lim["max_crab_speed"]))
        p("steer_rate", math.radians(cfg["steering"]["max_rate_deg_s"]))

        linkage = self.get_parameter("linkage").value or None
        self.geometry = geometry_from_config(cfg, linkage)
        adaptive = cfg["steering"].get("adaptive", {})
        self.kin = FourWheelSteeringKinematics(
            self.geometry,
            adaptive_low_speed=float(adaptive.get("low_speed", 2.0)),
            adaptive_high_speed=float(adaptive.get("high_speed", 8.0)))

        self.mode = self._validated_mode(self.get_parameter("steering_mode").value)
        self.add_on_set_parameters_callback(self._on_params)

        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_time = None
        self.ref = [0.0, 0.0, 0.0]          # rate-limited (vx, vy, wz)
        self.steer_cmd = [0.0] * 4
        self.meas_steer: Optional[List[float]] = None
        self.meas_omega: Optional[List[float]] = None
        self.x = self.y = self.yaw = 0.0
        self.last_js_time = None

        if self.get_parameter("use_stamped_vel").value:
            self.create_subscription(TwistStamped, "cmd_vel", lambda m: self._on_twist(m.twist), 10)
        else:
            self.create_subscription(Twist, "cmd_vel", self._on_twist, 10)
        if AckermannDriveStamped is not None:
            self.create_subscription(AckermannDriveStamped, "ackermann_cmd", self._on_ackermann, 10)
        self.create_subscription(String, "~/steering_mode", self._on_mode_msg, 10)
        self.create_subscription(JointState, "joint_states", self._on_joint_state, 50)

        self.pub_steer = self.create_publisher(Float64MultiArray, "steering_position_controller/commands", 10)
        self.pub_wheel = self.create_publisher(Float64MultiArray, "wheel_velocity_controller/commands", 10)
        self.pub_odom = self.create_publisher(Odometry, "odom", 20)
        self.pub_twist = self.create_publisher(TwistStamped, "~/realised_twist", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_mode = self.create_publisher(String, "~/active_mode", latched)
        self.tf = TransformBroadcaster(self)

        self.dt = 1.0 / float(self.get_parameter("update_rate").value)
        self.create_timer(self.dt, self._update)
        self._publish_mode()
        self.get_logger().info(
            f"4WS controller up: wheelbase {self.geometry.wheelbase:.3f} m, kingpin track "
            f"{self.geometry.kingpin_track:.3f} m, max steer "
            f"{math.degrees(self.geometry.max_steer):.1f} deg, linkage {self.geometry.linkage}, "
            f"mode {self.mode}; min turning radius (counter) {self.kin.turning_radius('counter'):.2f} m")

    # ---------------------------------------------------------------- inputs
    def _validated_mode(self, mode: str) -> str:
        mode = str(mode).strip().lower()
        if mode not in MODES:
            self.get_logger().error(f"unknown steering mode '{mode}', keeping counter. Valid: {MODES}")
            return "counter"
        return mode

    def _on_params(self, params: List[Parameter]) -> SetParametersResult:
        for prm in params:
            if prm.name == "steering_mode":
                if str(prm.value).lower() not in MODES:
                    return SetParametersResult(successful=False, reason=f"valid modes: {MODES}")
                self.mode = str(prm.value).lower()
                self._publish_mode()
            if prm.name == "differential" and prm.value not in ("electronic", "locked"):
                return SetParametersResult(successful=False, reason="differential: electronic|locked")
        return SetParametersResult(successful=True)

    def _on_mode_msg(self, msg: String) -> None:
        self.set_parameters([Parameter("steering_mode", Parameter.Type.STRING, msg.data.strip().lower())])

    def _publish_mode(self) -> None:
        self.pub_mode.publish(String(data=self.mode))

    def _on_twist(self, t: Twist) -> None:
        self.cmd = (t.linear.x, t.linear.y, t.angular.z)
        self.cmd_time = self.get_clock().now()

    def _on_ackermann(self, msg: AckermannDriveStamped) -> None:
        """Front-equivalent steering angle -> twist for the active mode."""
        v, d = msg.drive.speed, msg.drive.steering_angle
        if self.mode == MODE_CRAB:
            self.cmd = (v * math.cos(d), v * math.sin(d), 0.0)
        else:
            x_icr = self.kin.icr_line_x(self.mode if self.mode != "free" else "counter", v)
            arm = self.geometry.front_axle_x - x_icr
            if self.mode == "rear":
                arm = self.geometry.rear_axle_x - x_icr  # angle refers to the rear axle
            wz = v * math.tan(d) / arm if abs(arm) > 1e-6 else 0.0
            self.cmd = (v, 0.0, wz)
        self.cmd_time = self.get_clock().now()

    def _on_joint_state(self, msg: JointState) -> None:
        idx = {n: i for i, n in enumerate(msg.name)}
        sj = self.get_parameter("steer_joints").value
        wj = self.get_parameter("wheel_joints").value
        if not all(n in idx for n in sj + wj) or len(msg.velocity) < len(msg.name):
            return
        self.meas_steer = [msg.position[idx[n]] for n in sj]
        self.meas_omega = [msg.velocity[idx[n]] for n in wj]
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if self.last_js_time is not None:
            dt = (stamp - self.last_js_time).nanoseconds * 1e-9
            if 0.0 < dt < 0.5:
                self._integrate(dt, msg.header.stamp)
        self.last_js_time = stamp

    # ---------------------------------------------------------------- update
    def _update(self) -> None:
        now = self.get_clock().now()
        timeout = self.get_parameter("cmd_timeout").value
        vx_t, vy_t, wz_t = self.cmd
        if self.cmd_time is None or (now - self.cmd_time).nanoseconds * 1e-9 > timeout:
            vx_t = vy_t = wz_t = 0.0

        gp = lambda n: float(self.get_parameter(n).value)  # noqa: E731
        vx_t = _clamp(vx_t, -gp("max_reverse_speed"), gp("max_speed"))
        vy_t = _clamp(vy_t, -gp("max_lateral_speed"), gp("max_lateral_speed"))
        wz_t = _clamp(wz_t, -gp("max_yaw_rate"), gp("max_yaw_rate"))

        # Acceleration limiting on forward speed (decel when |v| shrinks).
        dv = vx_t - self.ref[0]
        a_lim = gp("max_decel") if abs(vx_t) < abs(self.ref[0]) or vx_t * self.ref[0] < 0 else gp("max_accel")
        self.ref[0] += _clamp(dv, -a_lim * self.dt, a_lim * self.dt)
        self.ref[1] += _clamp(vy_t - self.ref[1], -gp("max_accel") * self.dt, gp("max_accel") * self.dt)
        self.ref[2] += _clamp(wz_t - self.ref[2], -2.0 * self.dt, 2.0 * self.dt)

        cmd = self.kin.inverse(self.mode, *self.ref, hold=self.steer_cmd,
                               differential=self.get_parameter("differential").value)
        # Knuckle slew-rate limit (one electric actuator per axle).
        max_step = gp("steer_rate") * self.dt
        target_steer = cmd.steer
        self.steer_cmd = [s + _clamp(t - s, -max_step, max_step)
                          for s, t in zip(self.steer_cmd, target_steer)]

        # Slow the wheels down while the knuckles have not reached the
        # commanded geometry, otherwise the tyres fight each other.
        lag_ref = self.meas_steer if self.meas_steer is not None else self.steer_cmd
        lag = max(abs(t - m) for t, m in zip(target_steer, lag_ref))
        lo = math.radians(gp("steer_error_slowdown_deg"))
        hi = math.radians(gp("steer_error_stop_deg"))
        scale = 1.0 if lag <= lo else max(0.0, 1.0 - (lag - lo) / max(hi - lo, 1e-6))
        omega = [w * scale for w in cmd.omega]

        self.pub_steer.publish(Float64MultiArray(data=self.steer_cmd))
        self.pub_wheel.publish(Float64MultiArray(data=omega))
        ts = TwistStamped()
        ts.header.stamp = now.to_msg()
        ts.header.frame_id = self.get_parameter("base_frame").value
        ts.twist.linear.x, ts.twist.linear.y, ts.twist.angular.z = cmd.twist
        self.pub_twist.publish(ts)

    # -------------------------------------------------------------- odometry
    def _integrate(self, dt: float, stamp) -> None:
        vx, vy, wz = self.kin.forward(self.meas_steer, self.meas_omega)
        # Exact integration over the step (constant twist arc).
        yaw_mid = self.yaw + 0.5 * wz * dt
        self.x += (vx * math.cos(yaw_mid) - vy * math.sin(yaw_mid)) * dt
        self.y += (vx * math.sin(yaw_mid) + vy * math.cos(yaw_mid)) * dt
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.get_parameter("odom_frame").value
        odom.child_frame_id = self.get_parameter("base_frame").value
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(0.5 * self.yaw)
        odom.pose.pose.orientation.w = math.cos(0.5 * self.yaw)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        # Planar odometry: large variance on the unobserved dimensions.
        pc = [0.0] * 36
        for i, v in enumerate((0.05, 0.05, 1e6, 1e6, 1e6, 0.03)):
            pc[i * 7] = v
        odom.pose.covariance = pc
        tc = [0.0] * 36
        for i, v in enumerate((0.02, 0.02, 1e6, 1e6, 1e6, 0.02)):
            tc[i * 7] = v
        odom.twist.covariance = tc
        self.pub_odom.publish(odom)

        if self.get_parameter("publish_tf").value:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = odom.child_frame_id
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(tf)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FourWsController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
