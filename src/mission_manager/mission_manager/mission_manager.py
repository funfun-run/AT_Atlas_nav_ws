"""
Mission Manager Node

提供 NavigateToZone action server，封装 Nav2 NavigateToPose 航点发送。
"""
import os
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from mission_manager.action import NavigateToZone
from geometry_msgs.msg import PoseStamped, Quaternion
from ament_index_python.packages import get_package_share_directory

from mission_manager.waypoint_loader import load_waypoints


class MissionManager(Node):
    """提供 NavigateToZone action，内部封装 NavigateToPose。"""

    def __init__(self) -> None:
        super().__init__("mission_manager")

        # 加载航点
        map_dir = get_package_share_directory('at_nav2')
        map_path = os.path.join(map_dir, 'maps', 'map.yaml')
        self.waypoints = load_waypoints(map_path)
        self.get_logger().info(f'加载了 {len(self.waypoints)} 个航点')

        # Nav2 NavigateToPose action client
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # NavigateToZone action server
        self._action_server = ActionServer(
            self, NavigateToZone, "navigate_to_zone",
            execute_callback=self._execute_nav_cb
        )

        self.get_logger().info("Mission Manager 已启动")

    def _make_pose(self, x: float, y: float) -> PoseStamped:
        """将 (x, y) 坐标转为 PoseStamped。"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = Quaternion(w=1.0)
        return pose

    async def _execute_nav_cb(self, goal_handle):
        """NavigateToZone action 执行回调。"""
        zone_name = goal_handle.request.zone_name
        self.get_logger().info(f'收到导航请求: {zone_name}')

        if zone_name not in self.waypoints:
            goal_handle.abort()
            result = NavigateToZone.Result()
            result.success = False
            result.message = f'未知区域: {zone_name}'
            return result

        cx, cy = self.waypoints[zone_name]
        target_pose = self._make_pose(cx, cy)

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            goal_handle.abort()
            result = NavigateToZone.Result()
            result.success = False
            result.message = 'NavigateToPose action server 不可用'
            return result

        self.get_logger().info(f'发送导航目标: {zone_name} -> ({cx:.2f}, {cy:.2f})')

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = target_pose

        future = self._nav_client.send_goal_async(nav_goal)
        goal_handle_nav = await future

        if not goal_handle_nav.accepted:
            goal_handle.abort()
            result = NavigateToZone.Result()
            result.success = False
            result.message = '导航目标被拒绝'
            return result

        result_future = goal_handle_nav.get_result_async()
        while not result_future.done():
            feedback_msg = NavigateToZone.Feedback()
            feedback_msg.distance_remaining = -1.0  # 导航中
            goal_handle.publish_feedback(feedback_msg)
            await self._sleep(0.5)

        nav_result = result_future.result()
        goal_handle.succeed()

        result = NavigateToZone.Result()
        result.success = True
        result.message = f'导航完成: {zone_name}'
        return result

    async def _sleep(self, seconds: float):
        """Async sleep helper."""
        from rclpy.task import Future
        fut = Future()
        self.create_timer(seconds, lambda: fut.set_result(None))
        await fut


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
