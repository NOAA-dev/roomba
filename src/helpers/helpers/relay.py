#!/usr/bin/env python3
import math
import rclpy
import numpy as np
from rclpy.time import Time
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped,Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler 
from nav_msgs.msg import Odometry
 
def _required_float(node, name):
    """Read a geometry parameter that must come from robot_dimensions.yaml.

    There is deliberately no default: robot geometry has a single source of
    truth (program_bringup/config/robot_dimensions.yaml), so a launch that
    forgets to load it fails loudly instead of silently using stale numbers.
    """
    node.declare_parameter(name, Parameter.Type.DOUBLE)
    try:
        value = node.get_parameter(name).value
    except Exception:
        value = None
    if value is None:
        raise RuntimeError(
            f"Parameter '{name}' is not set. Load "
            "program_bringup/config/robot_dimensions.yaml into this node "
            "(<param from=...> / parameters=[...]).")
    return float(value)


def limit_wheel_speeds(left, right, wheel_radius, max_wheel_speed):
    """Limit wheel commands (rad/s) so the faster wheel stays under max_wheel_speed (m/s).

    Both wheels are scaled by the same factor, so the turn radius is preserved
    (clipping one wheel alone would change it). Non-finite input stops the robot.
    max_wheel_speed <= 0 disables the limit. Returns (left, right, was_limited).
    """
    if not (math.isfinite(left) and math.isfinite(right)):
        return 0.0, 0.0, True
    if max_wheel_speed > 0.0:
        peak = max(abs(left), abs(right)) * wheel_radius
        if peak > max_wheel_speed:
            scale = max_wheel_speed / peak
            return left * scale, right * scale, True
    return left, right, False


class CMDRelayNode(Node): 
    def __init__(self):
        super().__init__("cmd_relay") 

        self.get_logger().info("cmd_relay node has been started")
        # Geometry comes from program_bringup/config/robot_dimensions.yaml
        self.L_ = _required_float(self, "robot.drive_wheel.track_width_m")
        self.R_ = _required_float(self, "robot.drive_wheel.radius_m")

        # Safety.
        #   limits.max_wheel_speed_m_s  per-wheel speed cap (robot_dimensions.yaml, sim and hardware)
        #   cmd_timeout_s               stop if no velocity command arrives for this long, 0 = off
        self.max_wheel_speed = _required_float(self, "limits.max_wheel_speed_m_s")
        if self.max_wheel_speed <= 0.0:
            raise RuntimeError("limits.max_wheel_speed_m_s must be > 0")
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_s").value)
        self.last_cmd_time = None
        self.cmd_active = False  # a non-zero command is currently being held

        self.x_cood = 0.0
        self.y_cood = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.prev_rightwheel_pose_ = None
        self.prev_leftwheel_pose_ = None
        self.prev_time = None

        self.conversion_matrix = np.array([[self.R_/2, self.R_/2],
                                          [self.R_/self.L_, -self.R_/self.L_]])
        
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "roomba"
        # Covariances (diag, 6x6 row-major). Without these robot_localization
        # treats the wheel odometry as (near-)perfect. Values are initial
        # estimates -- tune from logged straight-line / spin tests.
        pc = [0.0] * 36
        tc = [0.0] * 36
        pc[0] = pc[7] = 0.01                 # x, y   [m^2]
        pc[14] = pc[21] = pc[28] = 1e6       # z, roll, pitch: unobserved (planar)
        pc[35] = 0.05                        # yaw    [rad^2]
        tc[0] = 0.002                        # vx     [(m/s)^2]
        tc[7] = 1e6                          # vy: not observed (diff drive)
        tc[14] = tc[21] = tc[28] = 1e6
        tc[35] = 0.02                        # vyaw   [(rad/s)^2]
        self.odom_msg.pose.covariance = pc
        self.odom_msg.twist.covariance = tc

        self.joy_subscriber_ = self.create_subscription(Twist,"/cmd_vel_unstamped",self.genrate_cmd_vel,10)
        self.joy_in_subscriber_ = self.create_subscription(TwistStamped,"/joystick_input/stamped",self.joy_in_callback,10)
        self.subscriber_ = self.create_subscription(JointState,"joint_states",self.position_callback,10)

        self.cmd_vel_publisher_ = self.create_publisher(Float64MultiArray,"/simple_velocity_controller/commands",10)
        self.diff_drive_cmd_vel_publisher_ = self.create_publisher(TwistStamped,"/cmd_vel",10)
        self.joy_in_publisher_ = self.create_publisher(Twist,"/joystick_input",10)
        self.odom_publisher_ = self.create_publisher(Odometry,"/odometry",10)

        self.timer = self.create_timer(0.02,self.odom_publisher)
        self.watchdog_timer = self.create_timer(0.05,self.cmd_watchdog)
 
    def genrate_cmd_vel(self,msg: Twist):

        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        matrix = np.array([[linear_velocity],[angular_velocity]])
        wheel_velocities = np.dot(np.linalg.inv(self.conversion_matrix),matrix)
        right_wheel_velocity = wheel_velocities[0][0]
        left_wheel_velocity = wheel_velocities[1][0]

        left_wheel_velocity, right_wheel_velocity, limited = limit_wheel_speeds(
            left_wheel_velocity, right_wheel_velocity, self.R_, self.max_wheel_speed)
        if limited:
            self.get_logger().warn(
                f"Wheel command limited (max {self.max_wheel_speed} m/s per wheel, "
                f"requested v={linear_velocity:.2f} m/s, w={angular_velocity:.2f} rad/s)",
                throttle_duration_sec=2.0)

        self.last_cmd_time = self.get_clock().now()
        self.cmd_active = bool(abs(left_wheel_velocity) > 1e-6 or abs(right_wheel_velocity) > 1e-6)

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [left_wheel_velocity, right_wheel_velocity]

        self.cmd_vel_publisher_.publish(wheel_speed_msg)

        cmd_vel_stamped_msg = TwistStamped()
        cmd_vel_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_vel_stamped_msg.twist.linear.x = linear_velocity
        cmd_vel_stamped_msg.twist.angular.z = angular_velocity


    def cmd_watchdog(self):
        """Stop the wheels if a non-zero command stops being refreshed.

        Without this the last wheel command stays latched: the hardware
        interface keeps sending keepalives, so the firmware watchdog never trips.
        """
        if not self.cmd_active or self.cmd_timeout <= 0.0 or self.last_cmd_time is None:
            return
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self.cmd_active = False
            stop_msg = Float64MultiArray()
            stop_msg.data = [0.0, 0.0]
            self.cmd_vel_publisher_.publish(stop_msg)
            self.get_logger().warn(f"No velocity command for {age:.2f} s -- wheels stopped")

    def joy_in_callback(self,msg: TwistStamped):
        joy_in_msg = Twist()
        joy_in_msg = msg.twist
        self.joy_in_publisher_.publish(joy_in_msg)

    def odom_publisher(self):

        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.pose.pose.position.x = self.x_cood
        self.odom_msg.pose.pose.position.y = self.y_cood
        self.odom_msg.twist.twist.linear.x = self.v
        self.odom_msg.twist.twist.angular.z = self.w

        r = quaternion_from_euler(0, 0, self.theta)
        self.odom_msg.pose.pose.orientation.x = r[0]
        self.odom_msg.pose.pose.orientation.y = r[1]
        self.odom_msg.pose.pose.orientation.z = r[2]
        self.odom_msg.pose.pose.orientation.w = r[3]

        self.odom_publisher_.publish(self.odom_msg)
    
    def position_callback(self,msg: JointState):

        try:
            idx_l = msg.name.index("base_left_wheel_joint")
            idx_r = msg.name.index("base_right_wheel_joint")
        except ValueError:
            return
        
        left_wheel_pose = msg.position[idx_l]
        right_wheel_pose = msg.position[idx_r]
        
        current_time = Time.from_msg(msg.header.stamp)
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_rightwheel_pose_ = right_wheel_pose
            self.prev_leftwheel_pose_ = left_wheel_pose
            return

        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        if dt <= 0.0:
            return

        delta_right_wheel = right_wheel_pose - self.prev_rightwheel_pose_
        delta_left_wheel = left_wheel_pose - self.prev_leftwheel_pose_

        self.prev_rightwheel_pose_ = right_wheel_pose
        self.prev_leftwheel_pose_ = left_wheel_pose

        linear_velocity = (self.R_ * (delta_right_wheel + delta_left_wheel) / 2.0) / dt
        angular_velocity = (self.R_ * (delta_right_wheel - delta_left_wheel) / self.L_) / dt
        delta_theta = angular_velocity * dt

        self.x_cood += linear_velocity * np.cos(self.theta + delta_theta/2) * dt
        self.y_cood += linear_velocity * np.sin(self.theta + delta_theta/2) * dt
        self.theta += angular_velocity * dt
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        if len(msg.velocity) > max(idx_l, idx_r):
            vl = msg.velocity[idx_l]
            vr = msg.velocity[idx_r]
            self.v = self.R_ * (vr + vl) / 2.0
            self.w = self.R_ * (vr - vl) / self.L_
        else:
            self.v = linear_velocity
            self.w = angular_velocity




def main(args=None):
    rclpy.init(args=args)
    node = CMDRelayNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()