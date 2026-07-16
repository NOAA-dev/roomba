#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.time import Time
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped,Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler 
from nav_msgs.msg import Odometry
 
 
class CMDRelayNode(Node): 
    def __init__(self):
        super().__init__("cmd_relay") 

        self.get_logger().info("cmd_relay node has been started")
        self.declare_parameter("wheelseperation", 0.23)
        self.declare_parameter("wheelradius", 0.05)

        self.L_ = self.get_parameter("wheelseperation").value
        self.R_ = self.get_parameter("wheelradius").value

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

        self.joy_subscriber_ = self.create_subscription(Twist,"/cmd_vel_unstamped",self.genrate_cmd_vel,10)
        self.joy_in_subscriber_ = self.create_subscription(TwistStamped,"/joystick_input/stamped",self.joy_in_callback,10)
        self.subscriber_ = self.create_subscription(JointState,"joint_states",self.position_callback,10)

        self.cmd_vel_publisher_ = self.create_publisher(Float64MultiArray,"/simple_velocity_controller/commands",10)
        self.diff_drive_cmd_vel_publisher_ = self.create_publisher(TwistStamped,"/cmd_vel",10)
        self.joy_in_publisher_ = self.create_publisher(Twist,"/joystick_input",10)
        self.odom_publisher_ = self.create_publisher(Odometry,"/odometry",10)

        self.timer = self.create_timer(0.02,self.odom_publisher)
 
    def genrate_cmd_vel(self,msg: Twist):

        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        matrix = np.array([[linear_velocity],[angular_velocity]])
        wheel_velocities = np.dot(np.linalg.inv(self.conversion_matrix),matrix)
        right_wheel_velocity = wheel_velocities[0][0]
        left_wheel_velocity = wheel_velocities[1][0]

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [left_wheel_velocity, right_wheel_velocity]

        self.cmd_vel_publisher_.publish(wheel_speed_msg)

        cmd_vel_stamped_msg = TwistStamped()
        cmd_vel_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_vel_stamped_msg.twist.linear.x = linear_velocity
        cmd_vel_stamped_msg.twist.angular.z = angular_velocity


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

        self.v = linear_velocity
        self.w = angular_velocity




def main(args=None):
    rclpy.init(args=args)
    node = CMDRelayNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()