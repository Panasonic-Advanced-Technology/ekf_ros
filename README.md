[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
# ekf_ros
> [!NOTE]
>本リポジトリは、[コンピューター・サイエンス＆テクノロジ専門誌Interface2026年6月号](https://interface.cqpub.co.jp/magazine/202606/)に掲載の記事[「GNSS＋IMUによる車輪型ロボの自己位置推定」](https://interface.cqpub.co.jp/wp-content/uploads/if2606_101.pdf)のサンプルコードです。

## 概要


## ディレクトリ構成

| ディレクトリ | 説明 |
|:--|:--|
| `src/ekf_ros/ekf_ros` | EKF 本体、ROS 2 ノード、可視化ノードなどの Python 実装です。 |
| `src/ekf_ros/launch` | ノードと RViz を起動する launch ファイルです。 |
| `src/ekf_ros/config` | EKF の設定ファイルと RViz の設定ファイルです。 |
| `rosbag` | 動作確認に使う rosbag データです。 |

## 動作環境

- Ubuntu 24.04
- ROS 2 Jazzy

## 環境構築

ROS 2 Jazzy がインストールされた環境で以下を実行してください。

```bash
cd ~/colcon_ws/src
git clone https://github.com/Panasonic-Advanced-Technology/ekf_ros.git
cd ~/colcon_ws
colcon build --symlink-install
source install/setup.bash
```

## 実行方法

### 動作確認

プログラム実行

```bash
ros2 launch ekf_ros sample.launch.py
```

rosbag の再生

```bash
cd rosbag
ros2 bag play sample
```

### GNSS 単独 vs GNSS + IMU の比較

GNSS のみを使った位置推定と、GNSS + IMU 融合による位置推定を同時に実行し、
途中で GNSS が欠損する区間（デフォルトでは 55 番目の GNSS 受信から 1 秒間）を
再現して両者の挙動を比較できます。

プログラム実行

```bash
ros2 launch ekf_ros gnss_vs_gnss_and_imu.launch.py
```

別ターミナルで rosbag の再生と記録

```bash
cd rosbag
# 記録用
ros2 bag record --all -o result 
# 再生
ros2 bag play s_curve --clock
```

記録した rosbag を解析して軌跡を PNG 出力

```bash
cd analysis
python3 analysis.py
# trajectory.png が出力され、画面にも表示されます
```

出力例:

![trajectory](analysis/trajectory.png)

- 青点: GNSS の観測位置
- 赤点: GNSS + IMU 融合の推定位置
- 緑点: GNSS 単独の推定位置


## ライセンス

[MIT License](LICENSE)