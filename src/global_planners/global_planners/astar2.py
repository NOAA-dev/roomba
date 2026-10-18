#!/usr/bin/env python3
import heapq
import itertools
import math
import time

import numpy as np
import rclpy
from custom_interfaces.msg import Validatedmap, NodeEnableStates
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from scipy.ndimage import distance_transform_edt
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import euler_from_quaternion, quaternion_from_euler


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


class node_state:
    def __init__(self, x, y, g=0.0, parent=None):
        self.x = x
        self.y = y
        self.g = g
        self.parent = parent


class AStarNode(Node):
    def __init__(self):
        super().__init__("a_star")

        # ---- robot geometry (single source: robot_dimensions.yaml) ----
        footprint_length = _required_float(self, "robot.footprint.length_m")
        footprint_width = _required_float(self, "robot.footprint.width_m")
        axle_x = _required_float(self, "robot.drive_wheel.axle_x_m")

        # ---- search limits ----
        self.max_expansions = 1000000
        self.max_search_time = 2.0  # seconds
        # Weight of the costmap term (0-100 cells) relative to metric step cost.
        self.cost_weight = 0.01
        self.clearance_radius = 0.10
        self.comfort_margin = 0.30

        # ---- pose ----
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.yaw = 0.0
        self.have_pose = False

        # ---- map ----
        self.Grid_ = None
        self.cost_map_ = None
        self.clearance_map = None
        self.lethal_mask = None
        self.map_resolution_ = None
        self.map_origin_x_ = None
        self.map_origin_y_ = None
        self.neighbors = []
        self.counter = 0

        # ---- search state ----
        self.computing = False
        self.open_set_ = []
        self.closed_set_ = set()
        self.prev_goal = None
        self.Goal_ = None

        # ---- ROS interfaces (identical to the kinodynamic planner) ----
        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_sub = self.create_subscription(Validatedmap, "/validated_map", self.map_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, "/path", path_qos)
        self.tf_timer = self.create_timer(0.01, self.check_tf)

        self.enabled = False
        enable_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enable_sub = self.create_subscription(
            NodeEnableStates, "/manager/enable_states", self.enable_callback, enable_qos)

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def enable_callback(self, msg: NodeEnableStates):
        self.enabled = msg.a_star

    def map_callback(self, msg: Validatedmap):
        if self.computing:
            return
        if self.counter == msg.counter:
            return
        self.counter = msg.counter

        width = msg.map.info.width
        height = msg.map.info.height
        self.Grid_ = np.array(msg.map.data, dtype=np.int8).reshape(height, width)
        self.map_resolution_ = msg.map.info.resolution
        self.map_origin_x_ = msg.map.info.origin.position.x
        self.map_origin_y_ = msg.map.info.origin.position.y

        r = self.map_resolution_
        d = r * math.sqrt(2)
        self.neighbors = [
            (-1, 0, r), (1, 0, r), (0, -1, r), (0, 1, r),
            (-1, -1, d), (-1, 1, d), (1, -1, d), (1, 1, d),
        ]

        self.cost_map_, self.clearance_map, self.lethal_mask = self.costmap_generator(self.Grid_, hard_limit=self.clearance_radius, comfort_limit=self.clearance_radius + self.comfort_margin,)

    def check_tf(self):
        if not self.enabled:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "roomba", rclpy.time.Time(), timeout=Duration(seconds=0.2))
            self.x_pos = transform.transform.translation.x
            self.y_pos = transform.transform.translation.y
            q = transform.transform.rotation
            _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.have_pose = True
        except TransformException:
            pass

    # ------------------------------------------------------------------ #
    # Costmap
    # ------------------------------------------------------------------ #
    def costmap_generator(self, grid, hard_limit=0.10, comfort_limit=0.30,occupied_thresh=65, allow_unknown=True,unknown_cost=3.0, decay_power=2.0):
        if not (comfort_limit > hard_limit > 0.0):
            raise ValueError("Require comfort_limit > hard_limit > 0")

        unknown = grid < 0
        occupied = grid >= occupied_thresh
        obstacle_mask = occupied if allow_unknown else (occupied | unknown)

        if np.any(obstacle_mask):
            clearance_map = distance_transform_edt(~obstacle_mask, sampling=self.map_resolution_).astype(np.float32)
        else:
            clearance_map = np.full(grid.shape, np.inf, dtype=np.float32)

        lethal_mask = clearance_map <= hard_limit
        inflation_mask = (~lethal_mask) & (clearance_map < comfort_limit)

        costmap = np.zeros(grid.shape, dtype=np.float32)
        normalized_distance = (comfort_limit - clearance_map[inflation_mask]) / (comfort_limit - hard_limit)
        costmap[inflation_mask] = 99.0 * np.power(normalized_distance, decay_power)

        if allow_unknown:
            costmap[unknown & ~lethal_mask] += unknown_cost

        costmap = np.clip(costmap, 0, 99)
        costmap[lethal_mask] = 100

        return np.rint(costmap).astype(np.uint8), clearance_map, lethal_mask

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #
    def heuristic_cost(self, x, y):  # octile heuristic (metres)
        dx = abs(self.Goal_[0] - x)
        dy = abs(self.Goal_[1] - y)
        M = max(dx, dy)
        m = min(dx, dy)
        D = self.map_resolution_
        D2 = self.map_resolution_ * math.sqrt(2)
        return D * (M - m) + D2 * m

    def world_to_grid(self, x, y):
        gx = int(math.floor((x - self.map_origin_x_) / self.map_resolution_))
        gy = int(math.floor((y - self.map_origin_y_) / self.map_resolution_))
        return gx, gy

    def grid_to_world(self, gx, gy):
        # cell centre
        x = (gx + 0.5) * self.map_resolution_ + self.map_origin_x_
        y = (gy + 0.5) * self.map_resolution_ + self.map_origin_y_
        return x, y

    def collision_check(self, x, y):
        """x, y are grid indices. True = blocked."""
        if x < 0 or y < 0 or x >= self.Grid_.shape[1] or y >= self.Grid_.shape[0]:
            return True
        if self.Grid_[y, x] > 0:
            return True
        if self.lethal_mask[y, x]:  # within clearance_radius of an obstacle
            return True
        return False

    # ------------------------------------------------------------------ #
    # Search
    # ------------------------------------------------------------------ #
    def A_star(self, start, goal):
        self.open_set_ = []
        self.closed_set_ = set()
        tie = itertools.count()
        goal = tuple(goal)

        search_t0 = time.perf_counter()

        start_node = node_state(start[0], start[1], 0.0, None)
        heapq.heappush(self.open_set_, (0.0, next(tie), start_node))
        best_cost = {tuple(start): 0.0}

        expansions = 0

        while self.open_set_:
            # ---- breaking constraints ----
            if expansions >= self.max_expansions:
                self.get_logger().warn(f"Expansion cap ({self.max_expansions}) reached, aborting search.")
                break
            if time.perf_counter() - search_t0 >= self.max_search_time:
                self.get_logger().warn(
                    f"Search time limit ({self.max_search_time:.1f}s) reached after {expansions} expansions, aborting.")
                break

            _, _, current = heapq.heappop(self.open_set_)
            current_key = (current.x, current.y)
            if current_key in self.closed_set_:
                continue

            self.closed_set_.add(current_key)
            expansions += 1

            if current_key == goal:
                self.get_logger().info("Goal reached!")
                self.get_logger().info(
                    f"Expansions: {expansions}, Open set size: {len(self.open_set_)}, "
                    f"Closed set size: {len(self.closed_set_)}")
                path = []
                node = current
                while node is not None:
                    path.append((node.x, node.y))
                    node = node.parent
                return path[::-1]

            for dx, dy, dg in self.neighbors:
                x_new, y_new = current.x + dx, current.y + dy
                new_key = (x_new, y_new)

                if new_key in self.closed_set_:
                    continue
                if self.collision_check(x_new, y_new):
                    continue

                g_cost = current.g + dg + self.cost_weight * float(self.cost_map_[y_new, x_new])

                if new_key not in best_cost or g_cost < best_cost[new_key]:
                    best_cost[new_key] = g_cost
                    f_cost = g_cost + self.heuristic_cost(x_new, y_new)
                    heapq.heappush(self.open_set_, (f_cost, next(tie), node_state(x_new, y_new, g_cost, current)))

        self.get_logger().warn("No path found!")
        return None

    # ------------------------------------------------------------------ #
    # Goal handling / path publishing
    # ------------------------------------------------------------------ #
    def goal_callback(self, msg: PoseStamped):
        if not self.enabled:
            return
        if self.Grid_ is None:
            self.get_logger().warn("Map not received yet!")
            return
        if not self.have_pose:
            self.get_logger().warn("Robot pose (map->roomba TF) not received yet!")
            return

        goal_x, goal_y = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)
        self.Goal_ = (goal_x, goal_y)

        if self.prev_goal is not None and self.prev_goal == self.Goal_:
            return

        q = msg.pose.orientation
        _, _, goal_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        t0 = time.perf_counter()  # total planning time starts here
        self.computing = True
        try:
            if self.collision_check(goal_x, goal_y):
                self.get_logger().warn("Goal is in collision / out of map, no path planned.")
                return

            start = self.world_to_grid(self.x_pos, self.y_pos)
            if not (0 <= start[0] < self.Grid_.shape[1] and 0 <= start[1] < self.Grid_.shape[0]):
                self.get_logger().warn("Robot is outside the map, no path planned.")
                return

            path = self.A_star(start, self.Goal_)

            if path is not None:
                self.publish_path(path, goal_yaw)
                self.prev_goal = self.Goal_
        finally:
            self.computing = False
            elapsed = time.perf_counter() - t0
            self.get_logger().info(f"Path planning time: {elapsed:.6f} seconds")

    def publish_path(self, path, goal_yaw):
        """Convert grid path to world-frame Path with tangent headings
        (the old planner's poses carried theta, so the follower may use it)."""
        world = [self.grid_to_world(gx, gy) for gx, gy in path]

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        n = len(world)
        for i, (x, y) in enumerate(world):
            if i < n - 1:
                theta = math.atan2(world[i + 1][1] - y, world[i + 1][0] - x)
            else:
                theta = goal_yaw  # final pose takes the requested goal heading

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            q = quaternion_from_euler(0, 0, theta)
            pose_stamped.pose.orientation.x = q[0]
            pose_stamped.pose.orientation.y = q[1]
            pose_stamped.pose.orientation.z = q[2]
            pose_stamped.pose.orientation.w = q[3]
            path_msg.poses.append(pose_stamped)

        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AStarNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()