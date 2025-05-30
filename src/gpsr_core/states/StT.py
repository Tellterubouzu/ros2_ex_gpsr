#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import speech_recognition as sr


class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text')

        self.publisher = self.create_publisher(String, 'speech_text', 10)
        self.done_subscriber = self.create_subscription(
            String,
            'speech_done',
            self.done_callback,
            10
        )

        self.listening = True
        self.get_logger().info("🎤 SpeechToTextNode ready. Say something after each 'done' signal.")

        # 初回入力開始
        self.recognize_and_publish()

    def recognize_and_publish(self):
        if not self.listening:
            return

        recognizer = sr.Recognizer()
        mic = sr.Microphone()

        with mic as source:
            self.get_logger().info("🔊 音声入力を待機中...（話してください）")
            recognizer.adjust_for_ambient_noise(source)
            audio = recognizer.listen(source)

        try:
            # Google Web Speech APIを使って音声認識（日本語）
            text = recognizer.recognize_google(audio, language='ja-JP')
            self.get_logger().info(f"✅ 音声認識結果: {text}")

            msg = String()
            msg.data = text
            self.publisher.publish(msg)

            # 次の音声入力は発話完了通知を待ってから
            self.listening = False

        except sr.UnknownValueError:
            self.get_logger().warn("⚠️ 音声を認識できませんでした。")
            self.recognize_and_publish()
        except sr.RequestError as e:
            self.get_logger().error(f"❌ Google音声認識サービスに接続できません: {e}")

    def done_callback(self, msg):
        if msg.data == "done":
            self.get_logger().info("🔁 発話完了を受信。次の音声入力に進みます。")
            self.listening = True
            self.recognize_and_publish()


def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
