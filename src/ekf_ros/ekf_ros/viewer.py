"""
ekf_viewer.py - EKF推定結果とGNSS観測の可視化ノード
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray


class EKFViewer(Node):
    """EKF推定軌跡とGNSS観測軌跡をRVizで可視化するノード"""

    # デフォルトパラメータ
    DEFAULT_PARAMS = {
        'enu_gnss_pose_topic': '/enu_gnss_pose',
        'estimated_pose_topic': '/estimated_pose',
        'frame_id': 'map',
        'gnss_marker_topic': '/gnss_markerarray',
        'ekf_marker_topic': '/ekf_markerarray',
    }

    # マーカー設定
    GNSS_MARKER_CONFIG = {
        'type': Marker.SPHERE,
        'scale': (0.3, 0.3, 0.3),
        'color': (0.0, 0.0, 1.0, 1.0),  # 青
    }

    EKF_MARKER_CONFIG = {
        'type': Marker.ARROW,
        'scale': (0.6, 0.08, 0.08),
        'color': (1.0, 0.0, 0.0, 1.0),  # 赤
    }

    def __init__(self):
        super().__init__('ekf_viewer')

        # パラメータ宣言・取得
        self._declare_parameters()
        self._load_parameters()

        # マーカー履歴
        self._gnss_marker_history = []
        self._ekf_marker_history = []

        # サブスクライバ
        self._gnss_sub = self.create_subscription(
            PoseStamped,
            self._enu_gnss_pose_topic,
            self._gnss_callback,
            10
        )
        self._ekf_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self._estimated_pose_topic,
            self._ekf_callback,
            10
        )

        # パブリッシャ
        self._gnss_marker_pub = self.create_publisher(
            MarkerArray, self._gnss_marker_topic, 1
        )
        self._ekf_marker_pub = self.create_publisher(
            MarkerArray, self._ekf_marker_topic, 1
        )

        self.get_logger().info('EKFViewer initialized')

    def _declare_parameters(self):
        """パラメータを宣言"""
        for name, default in self.DEFAULT_PARAMS.items():
            self.declare_parameter(name, default)

    def _load_parameters(self):
        """パラメータを読み込み"""
        self._frame_id = self._get_string_param('frame_id')
        self._enu_gnss_pose_topic = self._get_string_param('enu_gnss_pose_topic')
        self._estimated_pose_topic = self._get_string_param('estimated_pose_topic')
        self._gnss_marker_topic = self._get_string_param('gnss_marker_topic')
        self._ekf_marker_topic = self._get_string_param('ekf_marker_topic')

    def _get_string_param(self, name):
        """文字列パラメータを取得"""
        return self.get_parameter(name).get_parameter_value().string_value

    def _create_marker(self, ns, marker_id, config):
        """マーカーの基本設定を作成"""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self._frame_id
        marker.ns = ns
        marker.id = marker_id
        marker.type = config['type']
        marker.action = Marker.ADD

        # スケール設定
        marker.scale.x, marker.scale.y, marker.scale.z = config['scale']

        # 色設定
        r, g, b, a = config['color']
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        # 永続表示
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        return marker

    def _gnss_callback(self, msg):
        """GNSS観測コールバック - 青い球体で位置を表示"""
        marker = self._create_marker(
            ns='gnss_traj',
            marker_id=len(self._gnss_marker_history),
            config=self.GNSS_MARKER_CONFIG
        )

        # 位置設定
        marker.pose.position = msg.pose.position
        marker.pose.orientation.w = 1.0

        # 履歴に追加してパブリッシュ
        self._gnss_marker_history.append(marker)
        self._publish_markers(self._gnss_marker_pub, self._gnss_marker_history)

    def _ekf_callback(self, msg):
        """EKF推定コールバック - 赤い矢印で位置と姿勢を表示"""
        marker = self._create_marker(
            ns='ekf_traj',
            marker_id=len(self._ekf_marker_history),
            config=self.EKF_MARKER_CONFIG
        )

        # 位置・姿勢設定
        marker.pose = msg.pose.pose

        # 履歴に追加してパブリッシュ
        self._ekf_marker_history.append(marker)
        self._publish_markers(self._ekf_marker_pub, self._ekf_marker_history)

    def _publish_markers(self, publisher, markers):
        """MarkerArrayをパブリッシュ"""
        marker_array = MarkerArray()
        marker_array.markers = markers
        publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = EKFViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()