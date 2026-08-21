#!/usr/bin/env python3
import rclpy
import numpy as np
import math
from rclpy.node import Node
from rclpy.duration import Duration
from scipy import ndimage
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from custom_interfaces.msg import Validatedmap, NodeEnableStates


class FrontierExplorerNode(Node):
    def __init__(self):
        super().__init__("frontier_explorer")

        # sole authority for this is manager_node — default OFF
        self.enabled = False

        self.latest_valid_map = None
        self.last_seen_counter = -1

        self.x_pos = 0.0
        self.y_pos = 0.0

        self.current_goal = None
        self.goal_set_time = None

        self.goal_reached_radius = 0.75
        self.goal_timeout_sec = 14.0

        # minimum connected UNKNOWN region area (m^2) behind a frontier
        # cluster for it to be worth exploring — checked against the
        # CLEANED unknown mask, so sensor-noise checkerboard speckle near
        # range limits can't inflate an apparent "large" region
        self.min_frontier_area_m2 = 0.7 * 0.7

        # minimum clearance (meters) a goal point must have from the
        # nearest occupied cell — matches A*'s robot_radius + safety pad
        self.min_goal_clearance_m = 0.5

        self.push_cells = 14

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_sub = self.create_subscription(
            Validatedmap, "/validated_map", self.map_callback, 10)

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.exploration_done_pub = self.create_publisher(Bool, "/exploration_complete", 10)

        enable_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enable_sub = self.create_subscription(
            NodeEnableStates, "/manager/enable_states", self.enable_callback, enable_qos)

        self.tf_timer = self.create_timer(0.1, self.check_tf)
        self.plan_timer = self.create_timer(2.0, self.plan_next_frontier)

        self.get_logger().info("frontier_explorer active")

    def enable_callback(self, msg: NodeEnableStates):
        self.enabled = msg.frontier_explorer

    def map_callback(self, msg: Validatedmap):
        if msg.valid and msg.counter != self.last_seen_counter:
            self.last_seen_counter = msg.counter
            self.latest_valid_map = msg.map

    def check_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "roomba", rclpy.time.Time(), timeout=Duration(seconds=0.2))
            self.x_pos = transform.transform.translation.x
            self.y_pos = transform.transform.translation.y
        except TransformException as ex:
            self.get_logger().warn(f"TF unavailable: {ex}", throttle_duration_sec=2.0)

    def world_to_map(self, x, y, map_msg: OccupancyGrid):
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        res = map_msg.info.resolution
        return int((x - origin_x) / res), int((y - origin_y) / res)

    def map_to_world(self, col, row, map_msg: OccupancyGrid):
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        res = map_msg.info.resolution
        return origin_x + (col + 0.5) * res, origin_y + (row + 0.5) * res

    def clean_unknown_mask(self, unknown_mask: np.ndarray) -> np.ndarray:
        # sensor noise near the edge of range often produces a scattered
        # "checkerboard" of alternating free/unknown cells — this isn't a
        # real unexplored region, just noise. binary_opening strips out
        # isolated/thin unknown speckle while leaving genuinely large,
        # solid unknown regions intact.
        opened = ndimage.binary_opening(unknown_mask, structure=np.ones((3, 3)))
        closed = ndimage.binary_closing(opened, structure=np.ones((3, 3)))
        return closed

    def push_goal_into_known_space(self, centroid_row, centroid_col, reachable_free,
                                    unknown_clean, dist_from_occupied, min_clearance_cells):
        # distance (cells) from every free cell to the nearest unknown cell —
        # walking up this field's gradient moves the goal away from the
        # unknown boundary and into open known space, regardless of robot
        # position
        dist_from_unknown = ndimage.distance_transform_edt(~unknown_clean)

        height, width = reachable_free.shape
        row, col = centroid_row, centroid_col

        r0i, c0i = int(round(row)), int(round(col))
        r0i = min(max(r0i, 0), height - 1)
        c0i = min(max(c0i, 0), width - 1)
        best_point = (row, col)
        best_clearance = dist_from_occupied[r0i, c0i]

        for _ in range(self.push_cells):
            r_i, c_i = int(round(row)), int(round(col))

            r0, r1 = max(0, r_i - 1), min(height, r_i + 2)
            c0, c1 = max(0, c_i - 1), min(width, c_i + 2)
            window = dist_from_unknown[r0:r1, c0:c1]

            if window.size == 0:
                break

            local_max_idx = np.unravel_index(np.argmax(window), window.shape)
            target_r = r0 + local_max_idx[0]
            target_c = c0 + local_max_idx[1]

            if (target_r, target_c) == (r_i, c_i):
                break

            dr = target_r - row
            dc = target_c - col
            step_dist = math.hypot(dr, dc)
            if step_dist < 1e-3:
                break
            dr, dc = dr / step_dist, dc / step_dist

            next_row = row + dr
            next_col = col + dc
            nr_i, nc_i = int(round(next_row)), int(round(next_col))

            if not (0 <= nr_i < height and 0 <= nc_i < width):
                break
            if not reachable_free[nr_i, nc_i]:
                break

            row, col = next_row, next_col

            clearance = dist_from_occupied[nr_i, nc_i]
            if clearance > best_clearance:
                best_clearance = clearance
                best_point = (row, col)

            if clearance >= min_clearance_cells:
                return row, col, clearance

        return best_point[0], best_point[1], best_clearance

    def plan_next_frontier(self):
        if not self.enabled:
            return
        if self.latest_valid_map is None:
            return

        map_msg = self.latest_valid_map
        grid = np.array(map_msg.data, dtype=np.int8).reshape(
            map_msg.info.height, map_msg.info.width)
        resolution = map_msg.info.resolution
        cell_area = resolution * resolution
        min_clearance_cells = self.min_goal_clearance_m / resolution

        if self.current_goal is not None:
            gx, gy = self.current_goal
            dist_to_goal = math.hypot(gx - self.x_pos, gy - self.y_pos)
            elapsed = (self.get_clock().now() - self.goal_set_time).nanoseconds / 1e9

            if dist_to_goal <= self.goal_reached_radius:
                self.current_goal = None
            elif elapsed > self.goal_timeout_sec:
                self.get_logger().warn(
                    f"goal ({gx:.2f},{gy:.2f}) timed out after {elapsed:.1f}s, replanning")
                self.current_goal = None
            else:
                return

        free_mask = (grid == 0)
        unknown_mask_raw = (grid == -1)
        occupied_mask = (grid == 100)

        free_clean = ndimage.binary_opening(free_mask, structure=np.ones((3, 3)))
        unknown_clean = self.clean_unknown_mask(unknown_mask_raw)

        labeled, num_components = ndimage.label(free_clean, structure=np.ones((3, 3)))
        map_x, map_y = self.world_to_map(self.x_pos, self.y_pos, map_msg)
        height, width = labeled.shape
        if not (0 <= map_y < height and 0 <= map_x < width):
            self.get_logger().warn("robot position outside map bounds, skipping frontier search")
            return
        robot_label = labeled[map_y, map_x]
        if robot_label == 0:
            self.get_logger().warn("robot not on a labeled free cell, skipping this cycle")
            return
        reachable_free = (labeled == robot_label)

        unknown_dilated = ndimage.binary_dilation(unknown_clean, structure=np.ones((3, 3)))
        frontier_mask = reachable_free & unknown_dilated

        if not frontier_mask.any():
            self.get_logger().info("no frontiers left in reachable area — exploration complete")
            self.exploration_done_pub.publish(Bool(data=True))
            return

        frontier_labeled, num_frontiers = ndimage.label(frontier_mask, structure=np.ones((3, 3)))

        unknown_labeled, num_unknown = ndimage.label(unknown_clean, structure=np.ones((3, 3)))
        if num_unknown > 0:
            unknown_sizes = ndimage.sum(
                np.ones_like(unknown_clean, dtype=np.int32),
                unknown_labeled, range(1, num_unknown + 1))
        else:
            unknown_sizes = np.array([])

        occupied_dilated = ndimage.binary_dilation(occupied_mask, structure=np.ones((3, 3)))
        dist_from_occupied = ndimage.distance_transform_edt(~occupied_dilated)

        # nearest-first selection: among all frontier clusters that pass the
        # area + clearance checks, pick the CLOSEST one — makes exploration
        # proceed like an outward flood-fill instead of jumping to whichever
        # region happens to look largest
        best_dist = float("inf")
        best_world_xy = None

        for cluster_id in range(1, num_frontiers + 1):
            cluster_mask = (frontier_labeled == cluster_id)

            cluster_dilated = ndimage.binary_dilation(cluster_mask, structure=np.ones((3, 3)))
            bordering = unknown_labeled[cluster_dilated & unknown_clean]
            bordering = bordering[bordering > 0]
            if bordering.size == 0:
                continue
            max_unknown_cells = unknown_sizes[np.unique(bordering) - 1].max()
            if max_unknown_cells * cell_area < self.min_frontier_area_m2:
                continue

            rows, cols = np.nonzero(cluster_mask)
            centroid_row = rows.mean()
            centroid_col = cols.mean()

            centroid_row, centroid_col, clearance_cells = self.push_goal_into_known_space(
                centroid_row, centroid_col, reachable_free, unknown_clean,
                dist_from_occupied, min_clearance_cells)

            if clearance_cells < min_clearance_cells:
                continue

            world_x, world_y = self.map_to_world(centroid_col, centroid_row, map_msg)

            dist = math.hypot(world_x - self.x_pos, world_y - self.y_pos)
            if dist < 0.05:
                continue

            if dist < best_dist:
                best_dist = dist
                best_world_xy = (world_x, world_y)

        if best_world_xy is None:
            self.get_logger().info(
                "no frontiers with sufficient unknown area / clearance — exploration complete")
            self.exploration_done_pub.publish(Bool(data=True))
            return

        self.publish_goal(best_world_xy)

    def publish_goal(self, world_xy):
        world_x, world_y = world_xy
        self.current_goal = (world_x, world_y)
        self.goal_set_time = self.get_clock().now()

        yaw = math.atan2(world_y - self.y_pos, world_x - self.x_pos)
        q = quaternion_from_euler(0, 0, yaw)

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = world_x
        goal.pose.position.y = world_y
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        self.goal_pub.publish(goal)
        self.get_logger().info(f"new frontier goal: ({world_x:.2f}, {world_y:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
