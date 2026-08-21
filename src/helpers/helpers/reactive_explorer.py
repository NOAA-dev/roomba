#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from custom_interfaces.msg import Validatedmap, NodeEnableStates


class ReactiveExplorerNode(Node):
    def __init__(self):
        super().__init__("object_avoider")
        self.t = float()
        self.declare_parameter("min_distance", 1.2)
        self.declare_parameter("forward_speed", 1.0)
        self.min_distance = self.get_parameter("min_distance").value  # meters
        self.forward_speed = self.get_parameter("forward_speed").value  # meters/second

        self.keep_exploring = True
        self.once = False

        # sole authority for this is manager_node — default OFF
        self.enabled = False

        self.subscriber_ = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.subscriber_1 = self.create_subscription(Validatedmap, "/validated_map", self.map_validated, 10)
        self.publisher_ = self.create_publisher(Twist, "/autonomous_cmd_vel", 10)

        enable_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enable_sub = self.create_subscription(
            NodeEnableStates, "/manager/enable_states", self.enable_callback, enable_qos)

    def enable_callback(self, msg: NodeEnableStates):
        self.enabled = msg.reactive_explorer

    def map_validated(self, msg: Validatedmap):
        if msg.valid == True:
            self.keep_exploring = False
        else:
            self.keep_exploring = True

    def laser_callback(self, msg=LaserScan):
        if not self.enabled:
            return

        if self.keep_exploring == True:
            angle_min = msg.angle_min
            angle_inc = msg.angle_increment
            ranges = msg.ranges

            front_range = []
            right_range = []
            left_range = []

            angle = angle_min
            front_angle = np.deg2rad(30)
            angle_end = np.deg2rad(30)
            left_angle = np.deg2rad(120)
            right_angle = np.deg2rad(-120)
            for i, r in enumerate(ranges):
                if math.isfinite(r):
                    if (angle >= right_angle and angle <= -angle_end):
                        right_range.append(r)
                    if (angle >= -front_angle and angle <= front_angle):
                        front_range.append(r)
                    if (angle >= angle_end and angle <= left_angle):
                        left_range.append(r)
                angle += angle_inc

            front_clearance = min(front_range) if front_range else self.min_distance * 3
            right_clearance = min(right_range) if right_range else self.min_distance * 3
            left_clearance = min(left_range) if left_range else self.min_distance * 3

            cmd = Twist()

            safe_distance = 0.8
            max_speed = 0.8
            ratio = front_clearance / safe_distance
            if ratio >= 1.0:
                linear = max_speed * min(front_clearance / safe_distance, 0.5)
                angular = 0.0
            else:
                linear = 0.0
                error = left_clearance - right_clearance
                if error >= 0.0:
                    angular = 0.5
                else:
                    angular = -0.5

            cmd.linear.x = linear
            cmd.angular.z = angular

            self.publisher_.publish(cmd)
        if self.keep_exploring == False and self.once == False:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.publisher_.publish(cmd)
            self.once = True


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveExplorerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
