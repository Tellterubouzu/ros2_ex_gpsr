#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
task_planning.py – GPSR タスクプランナー (ROS 2 / OpenAI Chat API)
author: @Tellterubouzu
"""

import os, json
from typing import Dict, Any
from openai import OpenAI
import rclpy.logging

# ────────────────────────────────────────────────
# ステート一覧（ここに追加すれば自動でプロンプトへ反映）
# ────────────────────────────────────────────────
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


# ────────────────────────────────────────────────
# プロンプト
# ────────────────────────────────────────────────
_SYSTEM_PROMPT = (
    "You are the high-level task planner for a domestic service robot.\n"
    "Given the execution context (in JSON), return ONLY the name of the next state.\n"
    "Never output anything except one of the following valid state names:\n"
    + ", ".join(STATE_LIST)
)

_FEWSHOT = [
    # Init → TaskPlanning
    {"role": "user", "content": json.dumps({"previous_state": "Init"})},
    {"role": "assistant", "content": "TaskPlanning"},
    # TaskPlanning → WaitForHuman
    {"role": "user", "content": json.dumps({"previous_state": "TaskPlanning"})},
    {"role": "assistant", "content": "WaitForHuman"},
    # DetectHuman 認識失敗 → DetectHuman (リトライ)
    {
        "role": "user",
        "content": json.dumps(
            {"previous_state": "DetectHuman", "error_type": "recognition_failed"}
        ),
    },
    {"role": "assistant", "content": "DetectHuman"},
]

# ────────────────────────────────────────────────
# クラス
# ────────────────────────────────────────────────
class TaskPlanning:
    """OpenAI Chat API を呼び出して次ステート名を決定する"""

    def __init__(self, model: str = "gpt-4o-mini", temperature: float = 0.2, max_tokens: int = 8):
        if "OPENAI_API_KEY" not in os.environ:
            raise RuntimeError("環境変数 OPENAI_API_KEY が未設定です")
        self.client = OpenAI(api_key=os.environ["OPENAI_API_KEY"])
        self.model = model
        self.temperature = temperature
        self.max_tokens = max_tokens
        self.logger = rclpy.logging.get_logger("task_planning")

    def init(self) -> None:
        self.logger.info("TaskPlanner ready (model: %s)", self.model)

    def get_next_state(self, userdata: Dict[str, Any]) -> str:
        """LLM で次ステートを推論し、ステート名を返す"""
        messages = (
            [{"role": "system", "content": _SYSTEM_PROMPT}]
            + _FEWSHOT
            + [
                {
                    "role": "user",
                    "content": json.dumps(
                        {
                            "previous_state": userdata.get("previous_state", "Init"),
                            "speech_retry_count": userdata.get("speech_retry_count", 0),
                            "error_type": userdata.get("error_type"),
                        },
                        ensure_ascii=False,
                    ),
                }
            ]
        )

        try:
            resp = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=self.temperature,
                max_tokens=self.max_tokens,
            )
            next_state = resp.choices[0].message.content.strip()
        except Exception as e:
            self.logger.error("OpenAI API error or malformed output: %s", e)
            return "Error"

        if next_state not in STATE_LIST:
            self.logger.error("LLM returned unknown state: %s", next_state)
            return "Error"

        self.logger.info("LLM → %s", next_state)
        return next_state
    
if __name__ == "__main__":
    rclpy.init()
    task_planning = TaskPlanning()
    rclpy.spin(task_planning)
    rclpy.shutdown()