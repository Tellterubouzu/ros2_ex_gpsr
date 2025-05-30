#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import os
import tempfile
import subprocess

class TextToSpeechGttsNode(Node):
    def __init__(self):
        super().__init__('text_to_speech_gtts')

        self.subscription = self.create_subscription(
            String,
            'speech_text',
            self.speech_callback,
            10
        )

        self.done_publisher = self.create_publisher(
            String,
            'speech_done',
            10
        )

        self.get_logger().info("✅ TextToSpeechGttsNode is ready. Waiting for text on '/speech_text'...")

    def speech_callback(self, msg):
        text = msg.data
        self.get_logger().info(f"🗣 発話内容: {text}")

        try:
            # 一時ファイルを作成
            with tempfile.NamedTemporaryFile(delete=False, suffix='.mp3') as temp_file:
                file_path = temp_file.name

            # gTTSで音声ファイル生成（日本語指定）
            tts = gTTS(text=text, lang='ja')
            tts.save(file_path)

            # 音声を再生
            subprocess.run(['mpg123', file_path], check=True)

            # 再生後、ファイル削除
            os.remove(file_path)

            # 発話完了通知を送信
            done_msg = String()
            done_msg.data = "done"
            self.done_publisher.publish(done_msg)
            self.get_logger().info("✅ 発話完了: 'done' を /speech_done に送信しました")

        except Exception as e:
            self.get_logger().error(f"❌ gTTS 発話エラー: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TextToSpeechGttsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
