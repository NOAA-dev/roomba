#!/usr/bin/env python3
import rclpy
import numpy as np
import math
import cv2
import heapq
import time
from skimage.graph import MCP_Geometric
from custom_interfaces.msg import Validatedmap, NodeEnableStates
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.duration import Duration
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from scipy.ndimage import distance_transform_edt
from rclpy.qos import QoSProfile, DurabilityPolicy



class node_state:
    def __init__(self, x, y, g=0.0, parent=None):
        self.x = x
        self.y = y
        self.g = g
        self.parent = parent


class AStarNode(Node):
    def __init__(self):
        super().__init__("a_star")

        self.x_pos = 0.0
        self.y_pos = 0.0

        self.Grid_ = None
        self.cost_map_ = None
        self.clearance_map = None
        self.lethal_mask = None
        self.map_resolution_ = None
        self.map_origin_x_ = None
        self.map_origin_y_ = None
        
        r = self.map_resolution_
        self.neighbors = [
            (-1,  0, r),
            ( 1,  0, r),
            ( 0, -1, r),
            ( 0,  1, r),
            (-1, -1, r * math.sqrt(2)),
            (-1,  1, r * math.sqrt(2)),
            ( 1, -1, r * math.sqrt(2)),
            ( 1,  1, r * math.sqrt(2)),
        ]

        self.open_set_ = {}
        self.closed_set_ = set()

        self.clearance_radius = 0.25

        ###########################

    def map_callback(self, msg: Validatedmap):
        if self.counter == msg.counter:
            return
        self.counter = msg.counter
        width = msg.map.info.width
        height = msg.map.info.height
        self.Grid_ = np.array(msg.map.data, dtype=np.int8).reshape(height, width)
        self.map_resolution_ = msg.map.info.resolution
        self.map_origin_x_ = msg.map.info.origin.position.x
        self.map_origin_y_ = msg.map.info.origin.position.y
        self.cost_map_, self.clearance_map, self.lethal_mask = self.costmap_generator(self.Grid_)



    def costmap_generator(self, msg, hard_limit=0.10, comfort_limit=0.30, occupied_thresh=65, allow_unknown=True, unknown_cost=3.0, decay_power=2.0,):
        if not (comfort_limit > hard_limit > 0.0):
            raise ValueError("Require comfort_limit > hard_limit > 0")

        grid = msg
        unknown = grid < 0
        occupied = grid >= occupied_thresh

        # 2. Identify obstacles
        obstacle_mask = occupied if allow_unknown else (occupied | unknown)
        # 3. Calculate distance to the nearest obstacle (metres)
        if np.any(obstacle_mask):
            clearance_map = distance_transform_edt(~obstacle_mask,sampling=self.map_resolution_,).astype(np.float32)
        else:
            clearance_map = np.full((self.Grid_.shape[0], self.Grid_.shape[1]), np.inf, dtype=np.float32,)

        # 4. Assign lethal and inflation costs
        lethal_mask = clearance_map <= hard_limit
        inflation_mask = ((~lethal_mask) & (clearance_map < comfort_limit))

        costmap = np.zeros((self.Grid_.shape[0], self.Grid_.shape[1]), dtype=np.float32)

        normalized_distance = (comfort_limit - clearance_map[inflation_mask]) / (comfort_limit - hard_limit)

        costmap[inflation_mask] = (99.0 * np.power(normalized_distance, decay_power))

        # 5. Apply unknown-space penalty
        if allow_unknown:
            costmap[unknown & ~lethal_mask] += unknown_cost

        costmap = np.clip(costmap, 0, 99)
        costmap[lethal_mask] = 100

        return (np.rint(costmap).astype(np.uint8),clearance_map,lethal_mask,)

    def check_tf(self):
        if not self.enabled:
            return
        try:
            transform = self.tf_buffer.lookup_transform("map", "roomba", rclpy.time.Time(), timeout=Duration(seconds=0.2))

            self.x_pos = transform.transform.translation.x
            self.y_pos = transform.transform.translation.y
            z = transform.transform.translation.z
            q = transform.transform.rotation
            _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        except TransformException as ex:
            pass

    def heuristic_cost(self, x, y):  # octile heuristic
        M = max(abs(self.Goal_[0] - x), abs(self.Goal_[1] - y))
        m = min(abs(self.Goal_[0] - x), abs(self.Goal_[1] - y))
        D = self.map_resolution_
        D2 = self.map_resolution_ * math.sqrt(2)

        return (D * (M - m) + D2 * m)

    def world_to_grid(self, x, y):
        gx = int((x - self.map_origin_x_) / self.map_resolution_)
        gy = int((y - self.map_origin_y_) / self.map_resolution_)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = gx * self.map_resolution_ + self.map_origin_x_
        y = gy * self.map_resolution_ + self.map_origin_y_
        return x, y

    def collision_check(self, x, y):

        if x < 0 or y < 0 or x >= self.Grid_.shape[1] or y >= self.Grid_.shape[0]:
            return True
        if self.Grid_[y, x] > 0:
            return True
        if self.clearance_map[y, x] >= self.clearance_radius:
            return False
        if self.clearance_map[y, x] < self.clearance_radius:
            return True

        return False


    def A_star(self, start, goal):
        self.computing = True
        self.open_set_ = []
        self.closed_set_ = set()

        start_node = node_state(start[0], start[1], 0, None)

        heapq.heappush(self.open_set_, (0, id(start_node), start_node))

        best_cost = {}
        best_cost[start] = 0

        expansions = 0

        while self.open_set_:
            _, _, current = heapq.heappop(self.open_set_)
            current_key = (current.x, current.y)
            if current_key in self.closed_set_:
                continue

            self.closed_set_.add(current_key)
            expansions += 1
            if expansions >= self.max_expansions:
                break

            if (current.x, current.y) == tuple(self.Goal_):
                self.get_logger().info("Goal reached!")
                self.get_logger().info(f"Expansions: {expansions}, Open set size: {len(self.open_set_)}, Closed set size: {len(self.closed_set_)}")

                path = []
                node = current
                while node is not None:
                    path.append((node.x, node.y))
                    node = node.parent

                return path[::-1]

            for dx,dy,dg in self.neighbors:

                x_new, y_new = current.x + dx, current.y + dy

                collision = self.collision_check(x_new, y_new)
                if collision == True:
                    continue

                h_cost = self.heuristic_cost(x_new, y_new)
                g_cost = current.g + dg + self.cost_map_[y_new, x_new]

                new_node = node_state(x_new, y_new, g_cost, current)
                new_key = (x_new, y_new)

                if new_key not in best_cost or g_cost < best_cost[new_key]:
                    best_cost[new_key] = g_cost
                    f_cost = h_cost + g_cost
                    heapq.heappush(self.open_set_, (f_cost, id(new_node), new_node))

        self.get_logger().warn("No path found!")
        return None

    def goal_callback(self, msg: PoseStamped):
        if not self.enabled:
            return
        if self.Grid_ is None:
            self.get_logger().warn("Map not received yet!")
            return

        goal_x , goal_y = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)
        self.Goal_ = (goal_x, goal_y)


        t0 = time.perf_counter()
        start = self.world_to_grid(self.x_pos, self.y_pos)
        path = self.A_star(start, self.Goal_)

        elapsed = time.perf_counter() - t0
        self.get_logger().info(f"Path planning time: {elapsed:.6f} seconds")
        self.computing = False
        if path is not None:
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            for x, y in path:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "map"
                pose_stamped.pose.position.x = x
                pose_stamped.pose.position.y = y
                q = quaternion_from_euler(0, 0, 0)
                pose_stamped.pose.orientation.x = q[0]
                pose_stamped.pose.orientation.y = q[1]
                pose_stamped.pose.orientation.z = q[2]
                pose_stamped.pose.orientation.w = q[3]
                path_msg.poses.append(pose_stamped)

            self.path_pub.publish(path_msg)
            self.prev_goal = self.Goal_


def main(args=None):
    rclpy.init(args=args)
    node = AStarNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
