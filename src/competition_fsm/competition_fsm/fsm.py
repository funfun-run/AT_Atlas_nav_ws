"""竞赛状态机核心：Enum 状态定义 + 字典驱动 handler。"""
from enum import Enum
import logging


class FsmState(Enum):
    MANUAL = "manual"
    GO_TRANSIT = "go_transit"
    AT_TRANSIT = "at_transit"
    GO_DISPATCH = "go_dispatch"
    GO_ZONE_1 = "go_zone_1"
    GO_ZONE_2 = "go_zone_2"
    MISSION_DONE = "mission_done"


class CompetitionFsm:
    """比赛任务状态机。

    状态转移由两件事驱动：
    1. 导航到达 (on_arrived)
    2. 外部 /fsm_event service 调用 (on_fsm_event)
    """

    def __init__(self, logger: logging.Logger) -> None:
        self.logger = logger
        self.state = FsmState.MANUAL
        self.task_payload: str = ""

        # 每个状态的 handler
        self._handlers = {
            FsmState.MANUAL:        self._handle_manual,
            FsmState.GO_TRANSIT:    self._handle_go_transit,
            FsmState.AT_TRANSIT:    self._handle_at_transit,
            FsmState.GO_DISPATCH:   self._handle_go_dispatch,
            FsmState.GO_ZONE_1:     self._handle_go_zone,
            FsmState.GO_ZONE_2:     self._handle_go_zone,
            FsmState.MISSION_DONE:  self._handle_mission_done,
        }

        # (event_type, current_state) -> next_state 转移表
        self._event_transitions = {
            ("task_identified",  FsmState.AT_TRANSIT):  FsmState.GO_DISPATCH,
            ("pickup_complete",  FsmState.GO_DISPATCH): FsmState.GO_ZONE_1,
            ("delivery_complete", FsmState.GO_ZONE_1):  FsmState.GO_ZONE_2,
            ("delivery_complete", FsmState.GO_ZONE_2):  FsmState.MISSION_DONE,
        }

        # 外部回调（由 fsm_node 注入）
        self._on_enter_state = None   # callable(state): 进入状态时调用（发导航目标等）

    def switch_to(self, new_state: FsmState) -> None:
        """切换到新状态，执行 enter handler。"""
        old = self.state
        if old == new_state:
            return
        self.logger.info(f'[{old.value}] -> [{new_state.value}]')
        self.state = new_state
        handler = self._handlers.get(new_state)
        if handler:
            handler()

    def on_arrived(self) -> None:
        """导航到达的回调（由 fsm_node 在 action result 中调用）。"""
        self.logger.info(f'导航到达 (当前状态: {self.state.value})')

        # 只有 GO_TRANSIT 到达后自动转移
        if self.state == FsmState.GO_TRANSIT:
            self.switch_to(FsmState.AT_TRANSIT)
        # GO_DISPATCH, GO_ZONE_1, GO_ZONE_2 到达后等待外部 signal

    def on_fsm_event(self, event_type: str, payload: str = "") -> tuple[bool, str]:
        """处理外部 /fsm_event service 调用。

        Returns:
            (accepted, message) — 是否接受事件，拒绝原因
        """
        key = (event_type, self.state)
        next_state = self._event_transitions.get(key)

        if next_state is None:
            return False, f'状态 [{self.state.value}] 不接受事件 [{event_type}]'

        if event_type == "task_identified":
            self.task_payload = payload

        self.switch_to(next_state)
        return True, f'事件 [{event_type}] 已接受，转移到 [{next_state.value}]'

    # ── 各状态 handler ──

    def _handle_manual(self) -> None:
        self.logger.info('手动模式 — 等待切换指令')

    def _handle_go_transit(self) -> None:
        self.logger.info('导航到中转区')
        if self._on_enter_state:
            self._on_enter_state(FsmState.GO_TRANSIT)

    def _handle_at_transit(self) -> None:
        self.logger.info('等待任务识别...')

    def _handle_go_dispatch(self) -> None:
        self.logger.info('导航到待派送区')
        if self._on_enter_state:
            self._on_enter_state(FsmState.GO_DISPATCH)

    def _handle_go_zone(self) -> None:
        self.logger.info(f'导航到 {self.state.value}')
        if self._on_enter_state:
            self._on_enter_state(self.state)

    def _handle_mission_done(self) -> None:
        self.logger.info('全部任务完成!')
