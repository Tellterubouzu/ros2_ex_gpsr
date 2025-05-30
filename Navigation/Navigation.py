#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: navigation_report.py
"""

# モジュールのインポート（ROS2関連）
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
import tf_transformations
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action._navigate_to_pose import (
    NavigateToPose_GetResult_Response,
    NavigateToPose_Feedback,
    NavigateToPose_FeedbackMessage,
)
from action_msgs.msg import GoalStatus


class NavigationTurtlebot(Node):
    """NavigationSampleクラス（Nodeクラスを継承）
    指定した目的地にTurtleBot3を移動させるROS2ノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(引数は，'ノード名')
        super().__init__("navigation_report")

        # インスタンス変数の初期化
        self._goal_handle: ClientGoalHandle = None
        self._result_future: NavigateToPose_GetResult_Response = None
        self._feedback: NavigateToPose_Feedback = None
        self._status: int = None

        # ActionClientのインスタンスを生成（接続先のアクションサーバー名を指定）
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def goToPose(self, x: float, y: float, yaw: float) -> bool:
        """指定した目的地までナビゲーションするメソッド

        Args:
            x (float): 目的地のX座標[m]
            y (float): 目的地のY座標[m]
            yaw (float): 目的地での姿勢（Yaw角）[rad]

        Returns:
            bool: 'NavigateToPoseサーバーからの返答．True（承認），False（拒否）
        """

        self.get_logger().debug("Waiting for 'NavigateToPose' action server")

        # NavigateToPoseサーバーと接続できるまで待機
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'NavigateToPose' action server not available, waiting...")

        # PoseStampedメッセージを作成し，目的地の座標と姿勢を設定
        pose = PoseStamped()

        ## 目的地の座標を設定
        pose.header.stamp = self.get_clock().now().to_msg()  # 現在の時間をヘッダに設定
        pose.header.frame_id = "map"  # フレームIDを'map'に設定
        pose.pose.position.x = x  # 目的地のX座標[m]
        pose.pose.position.y = y  # 目的地のY座標[m]
        pose.pose.position.z = 0.0  # 目的地のz座標[m] # 2次元平面なので0.0

        ## 目的地での姿勢を設定
        ### オイラー角からクォータニオンへの変換
        quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[1]
        pose.pose.orientation.y = quat[2]
        pose.pose.orientation.z = quat[3]
        pose.pose.orientation.w = quat[0]

        # NavigateToPoseアクションのGoalメッセージを作成
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        # ログを表示
        self.get_logger().info(f"Navigating to goal: (x, y, yaw) = ({x}, {y}, {yaw})")

        # Goalメッセージをアクションサーバーに非同期で送信し，フィードバックメッセージ受信時に呼ぶコールバックメソッドを登録
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal=goal_msg,  # Goalメッセージ
            feedback_callback=self._feedbackcallback,  # コールバックメソッド
        )
        # Goalメッセージの送信が完了するまで待機
        rclpy.spin_until_future_complete(self, send_goal_future)

        # Goalメッセージの送信結果を取得
        self._goal_handle: ClientGoalHandle = send_goal_future.result()

        # NavigateToPoseサーバーの承認結果を確認
        if not self._goal_handle.accepted:
            self.get_logger().error(f"Goal to (x, y, yaw) = ({x}, {y}, {yaw}) was rejected!")
            return False

        # 非同期で結果を取得するように設定
        self._result_future = self._goal_handle.get_result_async()
        return True

    def cancelNav(self):
        """実行中のナビゲーションをキャンセルするメソッド"""
        self.get_logger().info("Canceling current task.")
        if self._result_future:
            future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)

    def isNavComplete(self) -> bool:
        """ナビゲーションの完了状態を確認するメソッド

        Returns:
            bool: ステータスを返す．True（キャンセル，完了），False（タイムアウト，処理中，未完了）
        """
        if not self._result_future:
            return True
        rclpy.spin_until_future_complete(self, self._result_future, timeout_sec=0.10)
        if self._result_future.result():
            self._status = self._result_future.result().status
            if self._status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().debug(f"Task with failed with status code: {self._status}")
                return True
        else:
            return False
        self.get_logger().debug(f"Navigation succeeded!")
        return True

    def getResult(self) -> int:
        """保留中のアクションの結果メッセージを取得するメソッド

        Returns:
            int: ステータスコードを返す．
        """
        return self._status

    def getFeedback(self) -> NavigateToPose_Feedback:
        """保留中のアクションのフィードバックメッセージを取得するメソッド

        Returns:
            NavigateToPose_Feedback: NavigateToPose_Feedbackオブジェクトを返す
        """
        return self._feedback

    def _feedbackcallback(self, msg: NavigateToPose_FeedbackMessage):
        """サーバーからのフィードバックを受信したときに呼ばれるコールバックメソッド

        Args:
            feedback_msg (NavigateToPose_FeedbackMessage): nav2_msgs/action/NavigateToPose型のFeedbackメッセージ
        """
        # フィードバックを取得
        self.get_logger().debug("Received action feedback message")
        self._feedback = msg.feedback


def main(args=None):
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # NavigationSampleクラスのインスタンス生成
    nav = NavigationTurtlebot()

    # 目的地の候補
    destination = {
        "1":   {"x": ???, "y": ???, "yaw": ???},
        "2":   {"x": ???, "y": ???, "yaw": ???},
        "3":   {"x": ???, "y": ???, "yaw": ???},
    }

    #目的地の場所を取得
    parser = argparse.ArgumentParser(description="Move to a named location")
    parser.add_argument("location", type=str, help="Location name (e.g., kitchen, entrance)")
    parsed_args = parser.parse_args(args)

    loc = parsed_args.location.lower() 

    #目的地の有無
    if loc not in destination:
        print(f"Error: Unknown location '{loc}'")
        print("Available locations:", ", ".join(destination.keys()))
        return

    # TODO 目的地の座標と目的地での姿勢を定義
    goal = destination[loc]
    nav.goToPose(x=goal["x"], y=goal["y"], yaw=goal["yaw"])  # 目的地

    # ナビゲーションは完了するまで待機
    while not nav.isNavComplete():
        # フィールドバックメッセージを取得
        feedback = nav.getFeedback()
        # ナビゲーション開始から600秒超えた場合はキャンセル
        if feedback.navigation_time > 600:
            nav.cancelNav()  # 実行中のナビゲーションをキャンセル

    # ナビゲーション結果を取得し，表示
    result = nav.getResult()
    match result:
        case GoalStatus.STATUS_SUCCEEDED:
            nav.get_logger().info("Navigation succeeded!")
            nav.destroy_node()
            rclpy.shutdown()
            return "success"
        case GoalStatus.STATUS_CANCELED:
            nav.get_logger().info("Navigation was canceled!")
        case GoalStatus.STATUS_ABORTED:
            nav.get_logger().error("Navigation failed!")
        case _:
            nav.get_logger().error("Unknown error!")

    # 終了処理
    nav.destroy_node()
    rclpy.shutdown()
    return "failure"


if __name__ == "__main__":
    import sys
    main(sys.argv[1:])

