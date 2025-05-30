#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import smach
import smach_ros
from navigation import Navigation
from object_recognition import ObjectRecognition
from speech_to_text import SpeechToText
from task_planning import TaskPlanning

#==================================================
## @file StateMachine.py
## @author Maiko Kudo
## @brief GPSRタスクを実行するROS2ノード
#==================================================

class State:
    def execute(self, userdata):
        raise NotImplementedError("execute()を実装してください")

class Init(State):
    def __init__(self):
        pass

    def execute(self, userdata):
        userdata["state_list"] = ["Init", "TaskPlanning", "WaitForHuman", "DetectHuman", "AskSelfIntroduction", "SpeechRetry", "CaptureFace", "NavigateToSeat", "IntroduceToOthers", "ObserveEnvironment", "ConfirmTaskComplete", "Error", "End"]
        userdata["speech_retry_limit"] = 3
        userdata["speech_retry_count"] = 0
        userdata["recoverable_error"] = True
        return "TaskPlanning"

class TaskPlanning(State):
    def __init__(self):
        pass

    def execute(self, userdata):
        return "WaitForHuman"

class WaitForHuman(State):
    def __init__(self):
        pass

    def execute(self, userdata):
        return "DetectHuman"

class DetectHuman(State):
    def __init__(self):
        pass

    def execute(self, userdata):
        return "AskSelfIntroduction"

class AskSelfIntroduction(State):
    def __init__(self):
        pass

    def execute(self, userdata):
        return "CaptureFace"

class SpeechRetry(State):
    def __init__(self):
        pass

    def execute(self, userdata):
        return "AskSelfIntroduction"

class CaptureFace(State):
    def __init__(self):
        pass

    def execute(self, userdata):
        return "NavigateToSeat"

class NavigateToSeat(State):
    def __init__(self):
        pass

    def execute(self, userdata):
        return "IntroduceToOthers"

class IntroduceToOthers(State):
    def __init__(self):
        pass

    def execute(self, userdata):
        return "ObserveEnvironment"

class ObserveEnvironment(State):
    def __init__(self):
        pass

    def execute(self, userdata):
        return "ConfirmTaskComplete"

class ConfirmTaskComplete(State):
    def __init__(self):
        pass

    def execute(self, userdata):
        return "TaskPlanning"

class Error(State):
    def __init__(self):
        pass

    def execute(self, userdata):
        error_type = userdata.get("error_type", "unknown")
        if error_type == "recognition_failed":
            userdata["recoverable_error"] = True
            return "DetectHuman"
        elif error_type == "speech_failed":
            userdata["recoverable_error"] = True
            return "AskSelfIntroduction"
        elif error_type in ["recognition_error", "speech_error"]:
            userdata["recoverable_error"] = False
            return "End"
        else:
            userdata["recoverable_error"] = True
            return "TaskPlanning"

class End(State):
    def __init__(self):
        pass

    def execute(self, userdata):
        return None

class StateMachine(Node):
    def __init__(self):
        super().__init__('gpsr_state_machine')
        
        # ステートマシンの宣言
        self._ssm = smach.StateMachine(outcomes=['exit'])
        
        with self._ssm:
            # 初期化ステート
            smach.StateMachine.add(
                'Init',
                Init(),
                transitions={
                    'next': 'TaskPlanning',
                    'error': 'Error'
                }
            )
            
            # タスク計画ステート
            smach.StateMachine.add(
                'TaskPlanning',
                TaskPlanning(),
                transitions={
                    'next': 'WaitForHuman',
                    'error': 'Error'
                }
            )
            
            # 人待ちステート
            smach.StateMachine.add(
                'WaitForHuman',
                WaitForHuman(),
                transitions={
                    'next': 'DetectHuman',
                    'error': 'Error'
                }
            )
            
            # 人検知ステート
            smach.StateMachine.add(
                'DetectHuman',
                DetectHuman(),
                transitions={
                    'next': 'AskSelfIntroduction',
                    'error': 'Error'
                }
            )
            
            # 自己紹介要求ステート
            smach.StateMachine.add(
                'AskSelfIntroduction',
                AskSelfIntroduction(),
                transitions={
                    'next': 'CaptureFace',
                    'error': 'Error'
                }
            )
            
            # 顔画像取得ステート
            smach.StateMachine.add(
                'CaptureFace',
                CaptureFace(),
                transitions={
                    'next': 'NavigateToSeat',
                    'error': 'Error'
                }
            )
            
            # 座席案内ステート
            smach.StateMachine.add(
                'NavigateToSeat',
                NavigateToSeat(),
                transitions={
                    'next': 'IntroduceToOthers',
                    'error': 'Error'
                }
            )
            
            # 紹介ステート
            smach.StateMachine.add(
                'IntroduceToOthers',
                IntroduceToOthers(),
                transitions={
                    'next': 'ObserveEnvironment',
                    'error': 'Error'
                }
            )
            
            # 環境観察ステート
            smach.StateMachine.add(
                'ObserveEnvironment',
                ObserveEnvironment(),
                transitions={
                    'next': 'ConfirmTaskComplete',
                    'error': 'Error'
                }
            )
            
            # タスク完了確認ステート
            smach.StateMachine.add(
                'ConfirmTaskComplete',
                ConfirmTaskComplete(),
                transitions={
                    'next': 'TaskPlanning',
                    'error': 'Error'
                }
            )
            
            # エラー処理ステート
            smach.StateMachine.add(
                'Error',
                Error(),
                transitions={
                    'recover': 'TaskPlanning',
                    'exit': 'exit'
                }
            )
        
        # ステートマシンの可視化サーバーを起動
        self._sis = smach_ros.IntrospectionServer('gpsr_state_machine', self._ssm, '/SM_ROOT')
        self._sis.start()

    def run(self):
        """ステートマシンを実行"""
        self._ssm.execute()

    def destroy(self):
        """リソースの解放"""
        self._sis.stop()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    state_machine = StateMachine()
    
    try:
        state_machine.run()
    finally:
        state_machine.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
