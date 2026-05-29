"""
Competition FSM Node

整合:
- FSM 状态机核心
- cmd_vel 仲裁（手动 vs Nav2）
- /fsm_event service（外部信号入口）
- NavigateToZone action client（调用 mission_manager）
"""
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from competition_fsm.srv import FsmEvent
from mission_manager.action import NavigateToZone

from competition_fsm.fsm import CompetitionFsm, FsmState


# zone_name to FsmState mapping
ZONE_TO_STATE = {
    FsmState.GO_TRANSIT:  "中转区",
    FsmState.GO_DISPATCH: "待派送区",
    FsmState.GO_ZONE_1:   "园区1",
    FsmState.GO_ZONE_2:   "园区2",
}


class CompetitionFsmNode(Node):
    """竞赛 FSM ROS2 节点。"""

    def __init__(self) -> None:
        super().__init__("competition_fsm")

        self.fsm = CompetitionFsm(self.get_logger())
        self.fsm._on_enter_state = self._on_enter_state
        self.arrived = False

        # ── cmd_vel 仲裁 ──
        # Nav2 controller_server 默认发 /cmd_vel
        self._nav2_cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self._nav2_cmd_cb, 10)
        # 遥控器发 /teleop_cmd_vel
        self._teleop_cmd_sub = self.create_subscription(
            Twist, "/teleop_cmd_vel", self._teleop_cmd_cb, 10)
        # FSM 仲裁后发给底盘的话题
        self._cmd_vel_pub = self.create_publisher(Twist, "/motor_cmd_vel", 10)
        self._last_teleop_time = self.get_clock().now()

        # 遥控超时 watchdog（1s 无消息 → 零速）
        self._teleop_watchdog = self.create_timer(0.2, self._teleop_watchdog_cb)

        # ── /fsm_event service ──
        self._fsm_srv = self.create_service(
            FsmEvent, "/fsm_event", self._on_fsm_event_cb)

        # ── /switch_mode topic ──
        self._switch_sub = self.create_subscription(
            String, "/switch_mode", self._on_switch_cmd, 10)

        # ── NavigateToZone action client ──
        self._nav_action_client = ActionClient(
            self, NavigateToZone, "navigate_to_zone")
        self._active_nav_goal = None

        self.get_logger().info("Competition FSM 已启动，当前: MANUAL")

    # ── 状态进入回调 ──

    def _on_enter_state(self, state: FsmState) -> None:
        """FSM 转移时触发：发送导航目标。"""
        self.arrived = False
        zone_name = ZONE_TO_STATE.get(state)
        if zone_name is None:
            return

        if not self._nav_action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('NavigateToZone action server 不可用')
            return

        goal = NavigateToZone.Goal()
        goal.zone_name = zone_name
        self.get_logger().info(f'发送导航目标: {zone_name}')
        future = self._nav_action_client.send_goal_async(goal)
        future.add_done_callback(self._nav_goal_response_cb)

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('导航目标被 mission_manager 拒绝')
            return
        self._active_nav_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        result = future.result()
        self._active_nav_goal = None
        if result.result.success:
            self.get_logger().info('导航到达目标')
            self.arrived = True
            self.fsm.on_arrived()
        else:
            self.get_logger().warn(f'导航失败: {result.result.message}')

    # ── cmd_vel 仲裁 ──

    def _nav2_cmd_cb(self, msg: Twist) -> None:
        """Nav2 controller 的 cmd_vel。只在 GO_* 状态转发。"""
        if self.fsm.state.value.startswith("go_"):
            self._cmd_vel_pub.publish(msg)

    def _teleop_cmd_cb(self, msg: Twist) -> None:
        """遥控器 cmd_vel。只在 MANUAL 状态转发。"""
        self._last_teleop_time = self.get_clock().now()
        if self.fsm.state == FsmState.MANUAL:
            self._cmd_vel_pub.publish(msg)

    def _teleop_watchdog_cb(self) -> None:
        """遥控超时保护：MANUAL 状态 1s 无遥控消息 → 零速。"""
        if self.fsm.state != FsmState.MANUAL:
            return
        elapsed = self.get_clock().now() - self._last_teleop_time
        if elapsed.nanoseconds * 1e-9 > 1.0:
            self._cmd_vel_pub.publish(Twist())  # 零速

    # ── /switch_mode ──

    def _on_switch_cmd(self, msg: String) -> None:
        """接收模式切换指令。

        "auto" → 手动切换到自动（如果当前 MANUAL）
        "manual" → 紧急切回 MANUAL
        """
        cmd = msg.data.strip().lower()
        if cmd == "auto" and self.fsm.state == FsmState.MANUAL:
            # 先刹车 0.5s，再切换
            self._cmd_vel_pub.publish(Twist())
            self.create_timer(0.5, lambda: self.fsm.switch_to(FsmState.GO_TRANSIT))
            self.get_logger().info('切换: MANUAL -> AUTO')

        elif cmd == "manual" and self.fsm.state != FsmState.MANUAL:
            self._cmd_vel_pub.publish(Twist())
            self.fsm.switch_to(FsmState.MANUAL)
            self.get_logger().info('切换: AUTO -> MANUAL')

    # ── /fsm_event service ──

    def _on_fsm_event_cb(self, request, response):
        """外部团队调用的 service 回调。"""
        accepted, message = self.fsm.on_fsm_event(
            request.event_type, request.payload)
        response.accepted = accepted
        response.message = message
        self.get_logger().info(
            f'/fsm_event: {request.event_type} -> accepted={accepted}')
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompetitionFsmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
