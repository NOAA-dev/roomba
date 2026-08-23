#!/usr/bin/env python3
"""
coverage_planner_node.py

ROS2 node that:
  1) Subscribes to a custom_interfaces/msg/ValidateMap message.
  2) Extracts the embedded nav_msgs/OccupancyGrid and runs Boustrophedon
     Cellular Decomposition (BCD) on it.
  3) Publishes a visualization_msgs/MarkerArray with ONE distinctly-colored
     marker per decomposed cell, ready to view in RViz.

Fixes applied to the original BCD implementation (it would not run as-is):
  - Cell.add_interval() appended to `self.slices`, which was never
    initialized -> now appends to `self.intervals` (the attribute that
    actually exists).
  - `intervals_connected` was declared as an instance method but missing
    `self`, yet called as `self.intervals_connected(...)` -> fixed to a
    proper staticmethod.
  - `decompose()` referenced `cell.id` / `child.id`, but Cell only has
    `.cell_id` -> all references fixed to `.cell_id`.
  - CONTINUE event called the non-existent `cell.add_slice(...)` -> fixed
    to `cell.add_interval(...)`.
"""

import colorsys
import heapq
from collections import defaultdict, deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

from custom_interfaces.msg import Validatedmap


# ======================================================================
# Boustrophedon Cellular Decomposition (bug-fixed)
# ======================================================================

class Cell:
    __slots__ = ("cell_id", "start_col", "end_col", "intervals",
                 "parents", "children", "closed")

    def __init__(self, cell_id, start_col):
        self.cell_id = cell_id
        self.start_col = start_col
        self.end_col = None
        self.intervals = []   # list of (col, y_start, y_end)
        self.parents = []
        self.children = []
        self.closed = False

    def add_interval(self, col, y_start, y_end):
        self.intervals.append((col, y_start, y_end))

    def close(self, col):
        self.end_col = col
        self.closed = True


class BoustrophedonCellularDecomposition:
    """
    Grid convention: grid[row=y, col=x]; 0 = free space, 1 = obstacle.
    Sweeps column by column and tracks free-space intervals, splitting /
    merging / opening / closing cells at critical points (classic BCD).
    """

    def __init__(self, occupancy_grid: np.ndarray):
        if occupancy_grid.ndim != 2:
            raise ValueError("Occupancy grid must be a 2D array (rows=y, cols=x).")
        self.grid = occupancy_grid
        self.height, self.width = occupancy_grid.shape

        self.cells = {}                    # cell_id -> Cell
        self._next_cell_id = 0
        self.adjacency = defaultdict(set)  # cell_id -> set(cell_id)

        self._active_cells = {}    # prev_interval_index -> cell_id
        self._prev_intervals = []  # list of (y_start, y_end)

    # ---------------- interval extraction ----------------
    def get_intervals(self, col_index):
        column = self.grid[:, col_index]
        intervals = []
        y = 0
        H = self.height
        while y < H:
            if column[y] == 0:
                y_start = y
                while y < H and column[y] == 0:
                    y += 1
                y_end = y - 1
                intervals.append((y_start, y_end))
            else:
                y += 1
        return intervals

    @staticmethod
    def intervals_connected(interval_a, interval_b):
        a_start, a_end = interval_a
        b_start, b_end = interval_b
        return max(a_start, b_start) <= min(a_end, b_end)

    def _connected_components(self, prev_intervals, curr_intervals):
        n_prev = len(prev_intervals)
        n_curr = len(curr_intervals)

        adj_prev_to_curr = defaultdict(set)
        adj_curr_to_prev = defaultdict(set)
        for i, pi in enumerate(prev_intervals):
            for j, cj in enumerate(curr_intervals):
                if self.intervals_connected(pi, cj):
                    adj_prev_to_curr[i].add(j)
                    adj_curr_to_prev[j].add(i)

        visited_prev, visited_curr = set(), set()
        components = []

        for i in range(n_prev):
            if i in visited_prev:
                continue
            comp_prev, comp_curr = set(), set()
            queue = deque([("p", i)])
            visited_prev.add(i)
            while queue:
                kind, idx = queue.popleft()
                if kind == "p":
                    comp_prev.add(idx)
                    for j in adj_prev_to_curr[idx]:
                        if j not in visited_curr:
                            visited_curr.add(j)
                            queue.append(("c", j))
                else:
                    comp_curr.add(idx)
                    for i2 in adj_curr_to_prev[idx]:
                        if i2 not in visited_prev:
                            visited_prev.add(i2)
                            queue.append(("p", i2))
            components.append((comp_prev, comp_curr))

        for j in range(n_curr):
            if j not in visited_curr:
                components.append((set(), {j}))

        return components

    # ---------------- cell bookkeeping ----------------
    def gen_new_cell(self, col, interval, parents=None):
        cell = Cell(self._next_cell_id, col)
        cell.add_interval(col, interval[0], interval[1])
        if parents:
            cell.parents = list(parents)
        self.cells[cell.cell_id] = cell
        self._next_cell_id += 1
        return cell

    def _close_cell(self, cell_id, col):
        cell = self.cells[cell_id]
        if not cell.closed:
            cell.close(col)

    def _link(self, id_a, id_b):
        if id_b not in self.adjacency[id_a]:
            self.adjacency[id_a].add(id_b)
            self.adjacency[id_b].add(id_a)

    # ---------------- main sweep ----------------
    def decompose(self):
        for col in range(self.width):
            intervals = self.get_intervals(col)

            if col == 0:
                new_active = {}
                for idx, interval in enumerate(intervals):
                    cell = self.gen_new_cell(col, interval)
                    new_active[idx] = cell.cell_id
                self._active_cells = new_active
                self._prev_intervals = intervals
                continue

            components = self._connected_components(self._prev_intervals, intervals)
            new_active = {}

            for comp_prev, comp_curr in components:
                n_p, n_c = len(comp_prev), len(comp_curr)

                if n_p == 0 and n_c == 1:                       # IN
                    (j,) = tuple(comp_curr)
                    cell = self.gen_new_cell(col, intervals[j])
                    new_active[j] = cell.cell_id

                elif n_p == 1 and n_c == 0:                      # OUT
                    (i,) = tuple(comp_prev)
                    cell_id = self._active_cells[i]
                    self._close_cell(cell_id, col - 1)

                elif n_p == 1 and n_c == 1:                      # CONTINUE
                    (i,) = tuple(comp_prev)
                    (j,) = tuple(comp_curr)
                    cell_id = self._active_cells[i]
                    cell = self.cells[cell_id]
                    cell.add_interval(col, intervals[j][0], intervals[j][1])
                    new_active[j] = cell_id

                elif n_p == 1 and n_c > 1:                       # SPLIT
                    (i,) = tuple(comp_prev)
                    parent_id = self._active_cells[i]
                    children_js = sorted(comp_curr)
                    self._close_cell(parent_id, col - 1)
                    for j in children_js:
                        child = self.gen_new_cell(col, intervals[j], parents=[parent_id])
                        self._link(parent_id, child.cell_id)
                        new_active[j] = child.cell_id

                elif n_p > 1 and n_c == 1:                       # MERGE
                    parents_is = sorted(comp_prev)
                    parent_ids = [self._active_cells[i] for i in parents_is]
                    (j,) = tuple(comp_curr)
                    for pid in parent_ids:
                        self._close_cell(pid, col - 1)
                    child = self.gen_new_cell(col, intervals[j], parents=parent_ids)
                    for pid in parent_ids:
                        self._link(pid, child.cell_id)
                    new_active[j] = child.cell_id

                else:                                            # COMPLEX
                    parents_is = sorted(comp_prev)
                    children_js = sorted(comp_curr)
                    parent_ids = [self._active_cells[i] for i in parents_is]
                    for pid in parent_ids:
                        self._close_cell(pid, col - 1)
                    for j in children_js:
                        child = self.gen_new_cell(col, intervals[j], parents=parent_ids)
                        for pid in parent_ids:
                            self._link(pid, child.cell_id)
                        new_active[j] = child.cell_id

            self._active_cells = new_active
            self._prev_intervals = intervals

        for cell_id in self._active_cells.values():
            self._close_cell(cell_id, self.width - 1)

        return self.cells, self.adjacency

    # ---------------- cell sequencing ----------------
    @staticmethod
    def _cell_centroid(cell):
        """Grid-coordinate centroid (col, row), weighted by covered cells."""
        total_col, total_row, total_pts = 0.0, 0.0, 0
        for (col, y_start, y_end) in cell.intervals:
            n = y_end - y_start + 1
            mid_row = (y_start + y_end) / 2.0
            total_col += col * n
            total_row += mid_row * n
            total_pts += n
        if total_pts == 0:
            return (float(cell.start_col), 0.0)
        return (total_col / total_pts, total_row / total_pts)

    def sequence_cells(self, start_hint=None):
        """
        Order all cells for visiting via greedy frontier expansion
        (Prim's-style MST growth): repeatedly add whichever unvisited
        cell -- among all cells adjacent to the already-visited set --
        has the centroid closest to the visited cell it borders.

        This always respects the adjacency graph (never "teleports"
        through a wall the way a raw distance-sorted list could), while
        doing noticeably less backtracking than plain DFS traversal.

        start_hint: optional (col, row) grid coordinate. The starting
        cell is the one whose centroid is nearest to this point (e.g.
        the robot's current position / dock, in grid coordinates). If
        None, defaults to the cell with the smallest (col, row)
        centroid, i.e. roughly the top-left of the map.

        Returns: list of cell_ids in visit order. Disconnected map
        regions (rare, but possible with noisy/fragmented maps) are
        each swept internally and appended in centroid order.
        """
        if not self.cells:
            return []

        centroids = {cid: self._cell_centroid(c) for cid, c in self.cells.items()}

        def nearest_to(point, candidates):
            return min(
                candidates,
                key=lambda cid: (centroids[cid][0] - point[0]) ** 2
                + (centroids[cid][1] - point[1]) ** 2,
            )

        visited = set()
        order = []
        frontier = []  # heap of (squared_dist, cell_id)

        def push_neighbors(cid):
            cx, cy = centroids[cid]
            for nb in self.adjacency.get(cid, ()):
                if nb not in visited:
                    nx, ny = centroids[nb]
                    d = (nx - cx) ** 2 + (ny - cy) ** 2
                    heapq.heappush(frontier, (d, nb))

        def grow_from(start_id):
            visited.add(start_id)
            order.append(start_id)
            push_neighbors(start_id)
            while frontier:
                d, cid = heapq.heappop(frontier)
                if cid in visited:
                    continue
                visited.add(cid)
                order.append(cid)
                push_neighbors(cid)

        first_start = (
            nearest_to(start_hint, centroids.keys())
            if start_hint is not None
            else min(centroids, key=lambda cid: centroids[cid])
        )
        grow_from(first_start)

        # Sweep any remaining disconnected components.
        remaining = [cid for cid in self.cells if cid not in visited]
        while remaining:
            next_start = min(remaining, key=lambda cid: centroids[cid])
            grow_from(next_start)
            remaining = [cid for cid in self.cells if cid not in visited]

        return order


# ======================================================================
# ROS2 Node
# ======================================================================

class CoveragePlannerNode(Node):
    def __init__(self):
        super().__init__("coverage_planner")

        # ---- parameters ----
        self.declare_parameter("validate_map_topic", "/validate_map")
        self.declare_parameter("marker_topic", "/bcd/cell_markers")
        self.declare_parameter("occupied_thresh", 50)      # >= this -> obstacle
        self.declare_parameter("unknown_as_obstacle", True)  # -1 cells -> obstacle
        self.declare_parameter("marker_lifetime_sec", 0.0)   # 0 = forever
        self.declare_parameter("publish_labels", True)
        self.declare_parameter("only_if_valid", False)       # require msg.valid
        self.declare_parameter("use_start_hint", False)
        self.declare_parameter("start_hint_x", 0.0)  # world-frame, only used if use_start_hint
        self.declare_parameter("start_hint_y", 0.0)

        self.validate_map_topic = self.get_parameter("validate_map_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.occupied_thresh = int(self.get_parameter("occupied_thresh").value)
        self.unknown_as_obstacle = bool(self.get_parameter("unknown_as_obstacle").value)
        self.marker_lifetime_sec = float(self.get_parameter("marker_lifetime_sec").value)
        self.publish_labels = bool(self.get_parameter("publish_labels").value)
        self.only_if_valid = bool(self.get_parameter("only_if_valid").value)
        self.use_start_hint = bool(self.get_parameter("use_start_hint").value)
        self.start_hint_x = float(self.get_parameter("start_hint_x").value)
        self.start_hint_y = float(self.get_parameter("start_hint_y").value)

        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Transient local so a RViz instance started (or re-opened) after
        # this node has already published still receives the last
        # MarkerArray instead of an empty display.
        marker_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(
            Validatedmap, self.validate_map_topic, self.validate_map_callback, sub_qos
        )
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, marker_qos)

        # Tracks the last map counter we ran BCD on, so repeated
        # ValidateMap messages carrying the same map (counter unchanged)
        # don't trigger redundant decomposition + marker republishing.
        self._last_counter = None

        self.get_logger().info(
            f"coverage_planner up. Subscribed to '{self.validate_map_topic}', "
            f"publishing markers on '{self.marker_topic}' (transient local)."
        )

    # ------------------------------------------------------------
    def validate_map_callback(self, msg: Validatedmap):
        # Only recompute BCD when the map has actually changed (new
        # counter). Repeated messages for the same map are ignored.
        if self._last_counter is not None and msg.counter == self._last_counter:
            self.get_logger().debug(
                f"Map counter unchanged ({msg.counter}); skipping BCD recompute."
            )
            return

        if self.only_if_valid and not msg.valid:
            self.get_logger().warn(
                f"Skipping map #{msg.counter}: marked invalid (map_score={msg.map_score:.1f})."
            )
            return

        self._last_counter = msg.counter

        grid_msg = msg.map
        width = grid_msg.info.width
        height = grid_msg.info.height
        resolution = grid_msg.info.resolution
        origin = grid_msg.info.origin.position

        if width == 0 or height == 0:
            self.get_logger().warn("Received empty OccupancyGrid, skipping BCD.")
            return

        raw = np.array(grid_msg.data, dtype=np.int16).reshape((height, width))

        # Build binary grid: 0 = free, 1 = obstacle (BCD convention)
        binary_grid = np.zeros((height, width), dtype=np.uint8)
        binary_grid[raw >= self.occupied_thresh] = 1
        if self.unknown_as_obstacle:
            binary_grid[raw < 0] = 1

        bcd = BoustrophedonCellularDecomposition(binary_grid)
        cells, adjacency = bcd.decompose()

        self.get_logger().info(
            f"BCD complete on map #{msg.counter}: {len(cells)} cells generated "
            f"({width}x{height} grid, res={resolution:.3f})."
        )

        start_hint = None
        if self.use_start_hint:
            # Convert the world-frame hint into grid (col, row) coordinates.
            start_hint = (
                (self.start_hint_x - origin.x) / resolution,
                (self.start_hint_y - origin.y) / resolution,
            )

        order = bcd.sequence_cells(start_hint=start_hint)
        self.get_logger().info(f"Cell visit order: {order}")

        marker_array = self.build_marker_array(
            cells, order, resolution, origin, grid_msg.header.frame_id
        )
        self.marker_pub.publish(marker_array)

    # ------------------------------------------------------------
    def build_marker_array(self, cells, order, resolution, origin, frame_id):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Clear previously published markers first.
        clear_marker = Marker()
        clear_marker.header.frame_id = frame_id
        clear_marker.header.stamp = now
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        lifetime = rclpy.duration.Duration(seconds=self.marker_lifetime_sec).to_msg()
        visit_index = {cid: i for i, cid in enumerate(order)}
        world_centroids = {}  # cell_id -> (x, y), filled in below

        for cell in cells.values():
            r, g, b = self._color_for_id(cell.cell_id)

            cube_marker = Marker()
            cube_marker.header.frame_id = frame_id
            cube_marker.header.stamp = now
            cube_marker.ns = "bcd_cells"
            cube_marker.id = cell.cell_id
            cube_marker.type = Marker.CUBE_LIST
            cube_marker.action = Marker.ADD
            cube_marker.pose.orientation.w = 1.0
            cube_marker.scale.x = resolution
            cube_marker.scale.y = resolution
            cube_marker.scale.z = resolution * 0.5
            cube_marker.color = ColorRGBA(r=r, g=g, b=b, a=0.85)
            cube_marker.lifetime = lifetime

            sum_x, sum_y, n_pts = 0.0, 0.0, 0
            for (col, y_start, y_end) in cell.intervals:
                wx = origin.x + (col + 0.5) * resolution
                for row in range(y_start, y_end + 1):
                    wy = origin.y + (row + 0.5) * resolution
                    cube_marker.points.append(Point(x=wx, y=wy, z=0.0))
                    sum_x += wx
                    sum_y += wy
                    n_pts += 1

            if n_pts == 0:
                continue
            marker_array.markers.append(cube_marker)

            centroid_x = sum_x / n_pts
            centroid_y = sum_y / n_pts
            world_centroids[cell.cell_id] = (centroid_x, centroid_y)

            if self.publish_labels:
                seq = visit_index.get(cell.cell_id)
                label_text = f"cell {cell.cell_id} (#{seq})" if seq is not None else f"cell {cell.cell_id}"
                label_marker = Marker()
                label_marker.header.frame_id = frame_id
                label_marker.header.stamp = now
                label_marker.ns = "bcd_cell_labels"
                label_marker.id = cell.cell_id
                label_marker.type = Marker.TEXT_VIEW_FACING
                label_marker.action = Marker.ADD
                label_marker.pose.position.x = centroid_x
                label_marker.pose.position.y = centroid_y
                label_marker.pose.position.z = resolution
                label_marker.pose.orientation.w = 1.0
                label_marker.scale.z = max(resolution * 4.0, 0.15)
                label_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                label_marker.text = label_text
                label_marker.lifetime = lifetime
                marker_array.markers.append(label_marker)

        # Sequence path: a line connecting cell centroids in visit order,
        # so the greedy frontier-expansion ordering can be sanity-checked
        # visually before waypoints are generated on top of it.
        if len(order) > 1:
            path_marker = Marker()
            path_marker.header.frame_id = frame_id
            path_marker.header.stamp = now
            path_marker.ns = "bcd_cell_sequence"
            path_marker.id = 0
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.pose.orientation.w = 1.0
            path_marker.scale.x = max(resolution * 0.5, 0.02)
            path_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
            path_marker.lifetime = lifetime
            for cid in order:
                if cid in world_centroids:
                    x, y = world_centroids[cid]
                    path_marker.points.append(Point(x=x, y=y, z=resolution))
            marker_array.markers.append(path_marker)

        return marker_array

    @staticmethod
    def _color_for_id(cell_id):
        """Evenly-spaced distinct hues via the golden-ratio conjugate trick."""
        golden_ratio_conjugate = 0.61803398875
        hue = (cell_id * golden_ratio_conjugate) % 1.0
        r, g, b = colorsys.hsv_to_rgb(hue, 0.65, 0.95)
        return float(r), float(g), float(b)


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
