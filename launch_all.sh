#!/bin/bash

# 環境のセットアップ
source /opt/ros/humble/setup.bash
source install/setup.bash

# 各ノードの起動
echo "Starting GPSR State Machine..."
ros2 launch gpsr_state_machine state_machine.launch.py &

echo "Starting Navigation..."
ros2 launch gpsr_navigation navigation.launch.py &

echo "Starting Vision System..."
ros2 launch gpsr_vision vision.launch.py &

echo "Starting Speech System..."
ros2 launch gpsr_speech speech.launch.py &

echo "Starting Planning System..."
ros2 launch gpsr_planning planning.launch.py &

# すべてのプロセスの終了を待機
wait 