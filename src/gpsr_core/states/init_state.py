#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import smach
from navigation import Navigation
from object_recognition import ObjectRecognition
from speech_to_text import SpeechToText
from task_planning import TaskPlanning

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'error'])
        self.navigation = Navigation()
        self.object_recognition = ObjectRecognition()
        self.speech_to_text = SpeechToText()
        self.task_planning = TaskPlanning()

    def execute(self, userdata):
        self.get_logger().info("[Init] 初期化")
        try:
            self.navigation.init()
            self.object_recognition.init()
            self.speech_to_text.init()
            self.task_planning.init()
            return 'next'
        except Exception as e:
            self.get_logger().error(f"[Init] 初期化エラー: {str(e)}")
            return 'error' 