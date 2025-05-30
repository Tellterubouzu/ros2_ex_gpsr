#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from navigation import Navigation
from object_recognition import ObjectRecognition
from speech_to_text import SpeechToText
from task_planning import TaskPlanning

#==================================================
## @file StateMachine.py
## @author Maiko Kudo
## @brief GPSRタスクを実行するROSノード
#==================================================

class State:
    def execute(self, userdata):
        raise NotImplementedError("execute()を実装してください")

class Init(State):
    def __init__(self):
        self.navigation = Navigation()
        self.object_recognition = ObjectRecognition()
        self.speech_to_text = SpeechToText()
        self.task_planning = TaskPlanning()

    def execute(self, userdata):
        rospy.loginfo("[Init] 初期化")
        try:
            self.navigation.init()
            self.object_recognition.init()
            self.speech_to_text.init()
            self.task_planning.init()
            return "TaskPlanning"
        except Exception as e:
            rospy.logerr(f"[Init] 初期化エラー: {str(e)}")
            return "Error"

class TaskPlanning(State):
    def __init__(self):
        self.task_planner = TaskPlanning()

    def execute(self, userdata):
        rospy.loginfo("[TaskPlanning] 次のタスク決定")
        try:
            # LLM/プランニングAPIを使用して次の状態を決定
            next_state = self.task_planner.get_next_state(userdata)
            
            if next_state not in userdata["state_list"]:
                rospy.logwarn(f"[TaskPlanning] 未知のステート名: {next_state}")
                return "Error"
                
            return next_state
        except Exception as e:
            rospy.logerr(f"[TaskPlanning] エラー: {str(e)}")
            return "Error"

class WaitForHuman(State):
    def __init__(self):
        self.object_recognition = ObjectRecognition()
        self.retry_count = 0
        self.max_retries = 3

    def execute(self, userdata):
        rospy.loginfo("[WaitForHuman] 人の到着待ち")
        try:
            # 人検知処理
            human_detected = self.object_recognition.detect_human()
            if human_detected:
                return "DetectHuman"
            
            self.retry_count += 1
            if self.retry_count >= self.max_retries:
                rospy.logwarn("[WaitForHuman] 検出失敗が多すぎます")
                return "Error"
                
            return "WaitForHuman"
        except Exception as e:
            rospy.logerr(f"[WaitForHuman] エラー: {str(e)}")
            return "Error"

class DetectHuman(State):
    def __init__(self):
        self.object_recognition = ObjectRecognition()
        self.retry_count = 0
        self.max_retries = 3

    def execute(self, userdata):
        rospy.loginfo("[DetectHuman] 人を認識中")
        try:
            # 画像認識処理
            success = self.object_recognition.recognize_human()
            
            if success:
                self.retry_count = 0  # 成功したらリトライカウントをリセット
                return "AskSelfIntroduction"
            
            # 失敗時の処理
            self.retry_count += 1
            if self.retry_count >= self.max_retries:
                rospy.logerr("[DetectHuman] 認識失敗が多すぎます")
                userdata["error_type"] = "recognition_failed"
                return "Error"
            
            rospy.logwarn(f"[DetectHuman] 認識失敗: {self.retry_count}回目")
            return "DetectHuman"  # リトライ
            
        except Exception as e:
            rospy.logerr(f"[DetectHuman] エラー: {str(e)}")
            userdata["error_type"] = "recognition_error"
            return "Error"

class AskSelfIntroduction(State):
    def __init__(self):
        self.speech_to_text = SpeechToText()
        self.retry_count = 0
        self.max_retries = 3

    def execute(self, userdata):
        rospy.loginfo("[AskSelfIntroduction] 自己紹介を聞く")
        try:
            # 音声認識処理
            success = self.speech_to_text.recognize_introduction()
            
            if success:
                self.retry_count = 0  # 成功したらリトライカウントをリセット
                return "CaptureFace"
            
            # 失敗時の処理
            self.retry_count += 1
            if self.retry_count >= self.max_retries:
                rospy.logerr("[AskSelfIntroduction] 音声認識失敗が多すぎます")
                userdata["error_type"] = "speech_failed"
                return "Error"
            
            rospy.logwarn(f"[AskSelfIntroduction] 認識失敗: {self.retry_count}回目")
            return "AskSelfIntroduction"  # リトライ
            
        except Exception as e:
            rospy.logerr(f"[AskSelfIntroduction] エラー: {str(e)}")
            userdata["error_type"] = "speech_error"
            return "Error"

class SpeechRetry(State):
    def execute(self, userdata):
        rospy.loginfo("[SpeechRetry] 音声認識リトライ")
        retry_limit = userdata.get("speech_retry_limit", 3)
        count = userdata.get("speech_retry_count", 0)
        
        if count < retry_limit:
            userdata["speech_retry_count"] = count + 1
            return "AskSelfIntroduction"
        return "Error"

class CaptureFace(State):
    def __init__(self):
        self.object_recognition = ObjectRecognition()
        self.retry_count = 0
        self.max_retries = 3

    def execute(self, userdata):
        rospy.loginfo("[CaptureFace] 顔画像取得")
        try:
            success = self.object_recognition.capture_face()
            if success:
                return "NavigateToSeat"
            
            self.retry_count += 1
            if self.retry_count >= self.max_retries:
                rospy.logwarn("[CaptureFace] 顔画像取得失敗が多すぎます")
                return "Error"
                
            return "CaptureFace"
        except Exception as e:
            rospy.logerr(f"[CaptureFace] エラー: {str(e)}")
            return "Error"

class NavigateToSeat(State):
    def __init__(self):
        self.navigation = Navigation()
        self.retry_count = 0
        self.max_retries = 3

    def execute(self, userdata):
        rospy.loginfo("[NavigateToSeat] 指定座標まで案内")
        try:
            success = self.navigation.move_to_seat()
            if success:
                return "IntroduceToOthers"
            
            self.retry_count += 1
            if self.retry_count >= self.max_retries:
                rospy.logwarn("[NavigateToSeat] ナビゲーション失敗が多すぎます")
                return "Error"
                
            return "NavigateToSeat"
        except Exception as e:
            rospy.logerr(f"[NavigateToSeat] エラー: {str(e)}")
            return "Error"

class IntroduceToOthers(State):
    def execute(self, userdata):
        rospy.loginfo("[IntroduceToOthers] 新しい人を紹介")
        try:
            # 紹介処理
            return "ObserveEnvironment"
        except Exception as e:
            rospy.logerr(f"[IntroduceToOthers] エラー: {str(e)}")
            return "Error"

class ObserveEnvironment(State):
    def __init__(self):
        self.object_recognition = ObjectRecognition()

    def execute(self, userdata):
        rospy.loginfo("[ObserveEnvironment] 環境観察")
        try:
            # 環境観察処理
            return "ConfirmTaskComplete"
        except Exception as e:
            rospy.logerr(f"[ObserveEnvironment] エラー: {str(e)}")
            return "Error"

class ConfirmTaskComplete(State):
    def execute(self, userdata):
        rospy.loginfo("[ConfirmTaskComplete] タスク完了確認")
        try:
            # 完了確認処理
            return "TaskPlanning"
        except Exception as e:
            rospy.logerr(f"[ConfirmTaskComplete] エラー: {str(e)}")
            return "Error"

class Error(State):
    def execute(self, userdata):
        error_type = userdata.get("error_type", "unknown")
        rospy.loginfo(f"[Error] 例外リカバリ: {error_type}")
        
        # エラータイプに応じた処理
        if error_type == "recognition_failed":
            # 画像認識失敗時の処理
            userdata["recoverable_error"] = True
            return "DetectHuman"
        elif error_type == "speech_failed":
            # 音声認識失敗時の処理
            userdata["recoverable_error"] = True
            return "AskSelfIntroduction"
        elif error_type in ["recognition_error", "speech_error"]:
            # システムエラー時の処理
            userdata["recoverable_error"] = False
            return "End"
        else:
            # その他のエラー
            userdata["recoverable_error"] = True
            return "TaskPlanning"

class End(State):
    def execute(self, userdata):
        rospy.loginfo("[End] 終了処理")
        return None

class StateMachine:
    def __init__(self):
        self.states = {
            "Init": Init(),
            "TaskPlanning": TaskPlanning(),
            "WaitForHuman": WaitForHuman(),
            "DetectHuman": DetectHuman(),
            "AskSelfIntroduction": AskSelfIntroduction(),
            "SpeechRetry": SpeechRetry(),
            "CaptureFace": CaptureFace(),
            "NavigateToSeat": NavigateToSeat(),
            "IntroduceToOthers": IntroduceToOthers(),
            "ObserveEnvironment": ObserveEnvironment(),
            "ConfirmTaskComplete": ConfirmTaskComplete(),
            "Error": Error(),
            "End": End()
        }
        self.current_state = "Init"
        self.userdata = {
            "state_list": list(self.states.keys()),
            "speech_retry_limit": 3,
            "speech_retry_count": 0,
            "recoverable_error": True
        }

    def run(self):
        while not rospy.is_shutdown():
            state = self.states[self.current_state]
            next_state = state.execute(self.userdata)
            if next_state is None:
                break
            self.current_state = next_state

def main():
    rospy.init_node('gpsr_state_machine')
    state_machine = StateMachine()
    state_machine.run()

if __name__ == '__main__':
    main()
