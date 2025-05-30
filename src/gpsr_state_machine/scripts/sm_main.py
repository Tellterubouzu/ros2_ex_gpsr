#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import os

# パスの設定
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from StateMachine import StateMachine

def main():
    try:
        # ROSノードの初期化
        rospy.init_node('gpsr_state_machine', anonymous=True)
        rospy.loginfo("GPSR State Machine Node Started")

        # ステートマシンのインスタンス化と実行
        state_machine = StateMachine()
        state_machine.run()

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception")
        pass
    except Exception as e:
        rospy.logerr(f"Error: {str(e)}")
        sys.exit(1)

if __name__ == '__main__':
    main() 