#!/usr/bin/env python3
"""Minimal stand-in for manager_node in manual-test (RViz goal) mode.

Does only two things, then idles:
  1. brings slam_toolbox and map_saver_server up (configure -> activate),
  2. publishes a latched /manager/enable_states enabling map_validator, a_star and
     pure_pursuit (explorers stay off).
No exploration, saved-map resume or goal publishing: goals come from RViz on /goal_pose.
"""
import rclpy
from custom_interfaces.msg import NodeEnableStates
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


class ManualTestBringup(Node):
    def __init__(self):
        super().__init__('manual_test_bringup')
        self._pub = self.create_publisher(
            NodeEnableStates, '/manager/enable_states',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _call(self, client, request, timeout=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def activate(self, name, wait_sec=30.0):
        get = self.create_client(GetState, f'/{name}/get_state')
        change = self.create_client(ChangeState, f'/{name}/change_state')
        if not (get.wait_for_service(timeout_sec=wait_sec) and change.wait_for_service(timeout_sec=5.0)):
            self.get_logger().error(f'{name}: lifecycle services not available')
            return False
        res = self._call(get, GetState.Request())
        state = res.current_state.id if res else None
        steps = {State.PRIMARY_STATE_UNCONFIGURED: [Transition.TRANSITION_CONFIGURE,
                                                    Transition.TRANSITION_ACTIVATE],
                 State.PRIMARY_STATE_INACTIVE: [Transition.TRANSITION_ACTIVATE],
                 State.PRIMARY_STATE_ACTIVE: []}.get(state)
        if steps is None:
            self.get_logger().error(f'{name}: unexpected lifecycle state {state}')
            return False
        for transition in steps:
            req = ChangeState.Request()
            req.transition.id = transition
            result = self._call(change, req)
            if result is None or not result.success:
                self.get_logger().error(f'{name}: transition {transition} failed')
                return False
        self.get_logger().info(f'{name} active')
        return True

    def enable_nodes(self):
        msg = NodeEnableStates()
        msg.map_validator = True
        msg.a_star = True
        msg.pure_pursuit = True
        msg.frontier_explorer = False
        msg.reactive_explorer = False
        self._pub.publish(msg)
        self.get_logger().info('enabled: map_validator, a_star, pure_pursuit')


def main(args=None):
    rclpy.init(args=args)
    node = ManualTestBringup()
    try:
        node.activate('slam_toolbox')
        node.activate('map_saver_server')
        node.enable_nodes()
        rclpy.spin(node)  # keep the latched enable message alive for late subscribers
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
