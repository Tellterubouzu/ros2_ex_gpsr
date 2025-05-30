#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import smach
from task_planning import TaskPlanning

class TaskPlanning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'error'])
        self.task_planner = TaskPlanning()

    def execute(self, userdata):
        self.get_logger().info("[TaskPlanning] 次のタスク決定")
        try:
            # LLM/プランニングAPIを使用して次の状態を決定
            next_state = self.task_planner.get_next_state(userdata)
            
            if next_state not in userdata["state_list"]:
                self.get_logger().warn(f"[TaskPlanning] 未知のステート名: {next_state}")
                return 'error'
                
            return 'next'
        except Exception as e:
            self.get_logger().error(f"[TaskPlanning] エラー: {str(e)}")
            return 'error' 