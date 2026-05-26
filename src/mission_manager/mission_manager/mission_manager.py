"""
Mission Manager Node

Task scheduling and navigation action client for waypoint-based missions.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class MissionManager(Node):
    """Manages mission execution by dispatching navigation goals."""

    def __init__(self) -> None:
        super().__init__("mission_manager")

        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.get_logger().info("Mission Manager has been started.")

    def send_goal(self, pose: PoseStamped) -> None:
        """Send a navigation goal to the action server."""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        ).add_done_callback(self._goal_response_cb)

        self.get_logger().info("Navigation goal sent.")

    def _goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by server.")
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f"Distance remaining: {feedback.distance_remaining:.2f}"
        )

    def _result_cb(self, future) -> None:
        result = future.result().result
        self.get_logger().info(f"Navigation completed. Result: {result}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
