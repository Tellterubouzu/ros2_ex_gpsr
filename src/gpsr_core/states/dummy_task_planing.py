#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
task_planning_dummy.py – テスト用プランナー，STATE_LIST からランダムに 1 つ返す
author: @Tellterubouzu
"""

import random
import rclpy.logging
from typing import Dict, Any

# 本番用と同じステート一覧を使う
STATE_LIST = [
    "Init",
    "TaskPlanning",
    "WaitForHuman",
    "DetectHuman",
    "AskSelfIntroduction",
    "SpeechRetry",
    "CaptureFace",
    "NavigateToSeat",
    "IntroduceToOthers",
    "ObserveEnvironment",
    "ConfirmTaskComplete",
    "Error",
    "End",
]

class DummyTaskPlanning:
    """ランダムに次ステートを選ぶだけのダミー版"""

    def __init__(self, exclude: list[str] | None = None):
        """
        Parameters
        ----------
        exclude : list[str] | None
            選択肢から除外したいステート名のリスト
            (例: ["Init", "End"] など)
        """
        self.exclude = set(exclude or [])
        self.logger = rclpy.logging.get_logger("dummy_task_planning")

    def init(self) -> None:
        self.logger.info("DummyTaskPlanner ready (random choice)")

    def get_next_state(self, userdata: Dict[str, Any]) -> str:
        """STATE_LIST からランダムに 1 つ返す"""
        candidates = [s for s in STATE_LIST if s not in self.exclude]
        next_state = random.choice(candidates)
        self.logger.info("Dummy planner chose: %s", next_state)
        return next_state
    
if __name__ == "__main__":
    rclpy.init()
    dummy_task_planning = DummyTaskPlanning()
    rclpy.spin(dummy_task_planning)
    rclpy.shutdown()