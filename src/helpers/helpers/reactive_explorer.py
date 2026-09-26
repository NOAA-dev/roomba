#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from custom_interfaces.msg import Validatedmap, NodeEnableStates


class ReactiveExplorerNode(Node):
    def __init__(self):
        super().__init__("object_avoider")
        self.t = float()
        self.declare_parameter("min_distance", 1.2)
        self.declare_parameter("forward_speed", 1.0)
        # LIDAR is mounted 0.1m forward (+x) of base_link/robot center (see
        # base_lidar_joint in roomba.urdf.xacro). Raw scan ranges are
        # distances FROM THE SENSOR, not from robot center - a beam
        # straight ahead reads ~0.1m shorter than the obstacle's true
        # distance from center, and off-axis beams need the full (x,y)
        # correction, not just a flat offset. See _range_from_center().
        self.declare_parameter("lidar_offset_x", 0.1)
        self.min_distance = self.get_parameter("min_distance").value  # meters
        self.forward_speed = self.get_parameter("forward_speed").value  # meters/second
        self.lidar_offset_x = self.get_parameter("lidar_offset_x").value

        self.keep_exploring = True
        self.once = False

        # sole authority for this is manager_node — default OFF
        self.enabled = False

        self.subscriber_ = self.create_subscription(LaserScan, "/scan", self.laser_callback, qos_profile_sensor_data)
        self.subscriber_1 = self.create_subscription(Validatedmap, "/validated_map", self.map_validated, 10)
        self.publisher_ = self.create_publisher(Twist, "/autonomous_cmd_vel", 10)

        enable_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enable_sub = self.create_subscription(
            NodeEnableStates, "/manager/enable_states", self.enable_callback, enable_qos)

    def enable_callback(self, msg: NodeEnableStates):
        self.enabled = msg.reactive_explorer

    def _range_from_center(self, r, angle):
        """Convert a raw range+angle (in the LIDAR's own frame) into the
        equivalent distance from base_link/robot center, accounting for
        the LIDAR sitting lidar_offset_x forward of center. The sensor-
        to-base mounting has no rotation (see roomba.urdf.xacro), only
        this translation, so a beam's Cartesian point in the sensor frame
        is just shifted by (lidar_offset_x, 0) to land in the base frame.
        """
        x = r * math.cos(angle) + self.lidar_offset_x
        y = r * math.sin(angle)
        return math.hypot(x, y)

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
            range_min = getattr(msg, "range_min", 0.05)
            for i, r in enumerate(ranges):
                # A LIDAR reports 0.0 (or a value below its own range_min)
                # for "no valid return" - out of range, absorbed, a
                # reflective surface, etc. That is NOT an obstacle;
                # math.isfinite(0.0) is True, so without the r > range_min
                # check every invalid return was being read as an obstacle
                # sitting right on the sensor, permanently winning the
                # sector min() over any real, farther object and making
                # front_clearance look ~0 almost every scan.
                if math.isfinite(r) and r > range_min:
                    r_center = self._range_from_center(r, angle)
                    if (angle >= right_angle and angle <= -angle_end):
                        right_range.append(r_center)
                    if (angle >= -front_angle and angle <= front_angle):
                        front_range.append(r_center)
                    if (angle >= angle_end and angle <= left_angle):
                        left_range.append(r_center)
                angle += angle_inc

            front_clearance = min(front_range) if front_range else self.min_distance * 3
            right_clearance = min(right_range) if right_range else self.min_distance * 3
            left_clearance = min(left_range) if left_range else self.min_distance * 3

            cmd = Twist()

            safe_distance = 0.4
            max_speed = 0.4
            ratio = front_clearance / safe_distance
            if ratio >= 1.0:
                linear = max_speed * min(front_clearance / safe_distance, 0.5)
                angular = 0.0
            else:
                linear = 0.0
                error = left_clearance - right_clearance
                if error >= 0.0:
                    angular = 0.3
                else:
                    angular = -0.3

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