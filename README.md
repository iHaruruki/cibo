# cibo
## Setup
1. Setup camera
Please follow link
[OrbbecSDK_ROS2](https://github.com/iHaruruki/OrbbecSDK_ROS2.git)

2. Setup python environment
Install pyrhon packages
```bash
pip3 install -U "numpy==1.26.4" "opencv-python==4.10.0.84"
pip3 install opencv-python mediapipe
```

3. Setup cibo Repositories
```bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/iHaruruki/cibo.git
```
Build
```bash
$ cd ~/ros2_ws
$ colcon build --symlink-install --packages-select cibo
$ source install/setup.bash
```

## How to use
- Run camera
```bash
ros2 launch orbbec_camera multi_camera.launch.py
```
- cibo.launch.py (front_camera_node & top_camera_node)
```bash
ros2 launch cibo cibo.launch.py
```
- ROI選択方法
1. ノード起動後、OpenCVウィンドウが表示されます
2. マウスをドラッグして長方形のROIを選択します
3. ドラッグ中は青い矩形が表示され、確定後は緑の矩形で表示されます

- 出力画像を見る(OpenCV Image Show) 
```bash
ros2 run cibo image_show_node
```
- Run chew count node
```bash
ros2 run cibo chew_counter_node
```

- rosbag
画像を録画したい場合は、rosbagを利用
```bash
cd ~/ros2_ws/rosbag
# もし作成して場合は mkdir -p rosbag
```
```bash
ros2 bag record -a
```
This command is mode that record all topic.

## Node List

### front_camera_node
- **説明**: フロントカメラ用の骨格推定ノード。Face Meshモデルによる詳細な顔解析（虹彩検出含む）を実行

### top_camera_node  
- **説明**: トップカメラ用の骨格推定ノード。ポーズと手の検出に特化

### chew_counter_node
- **説明**: 咀嚼回数をカウントする

計算方法  

MAR（Mouth Aspect Ratio）は以下の4点のランドマーク座標を使用して計算されます。  

| ランドマーク名 | MediaPipe Index | 説明 |
|----------------|------------------|------|
| 左口角 | 61 | 口の左端 |
| 右口角 | 291 | 口の右端 |
| 上唇中央 | 13 | 上唇の内側中央 |
| 下唇中央 | 14 | 下唇の内側中央 |

式：\
```math
$$ MAR = \frac{vertical distance (upper–lower lip)}{horizontal distance (left–right corner)} $$
```

MediaPipe Face Mesh の代表点で書くと：\
```math
$$ MAR = \frac{|P_{13} - P_{14}|}{|P_{61} - P_{291}|} $$
```

## Topic List

### front_camera_node

#### Subscribed Topics
| Topic名 | メッセージ型 | 説明 |
|---------|-------------|------|
| `/camera_02/color/image_raw` | `sensor_msgs/Image` | フロントカメラからの入力画像 |

#### Published Topics
| Topic名 | メッセージ型 | 説明 |
|---------|-------------|------|
| `/front_camera/annotated_image` | `sensor_msgs/Image` | ランドマーク付きの画像 |
| `/front_camera/pose_landmarks` | `std_msgs/Float32MultiArray` | ポーズランドマーク座標（x,y,z） |
| `/front_camera/face_landmarks` | `std_msgs/Float32MultiArray` | 顔ランドマーク座標（Face Meshモデル、最大478ポイント、虹彩含む） |
| `/front_camera/left_hand_landmarks` | `std_msgs/Float32MultiArray` | 左手ランドマーク座標（x,y,z） |
| `/front_camera/right_hand_landmarks` | `std_msgs/Float32MultiArray` | 右手ランドマーク座標（x,y,z） |

### top_camera_node

#### Subscribed Topics
| Topic名 | メッセージ型 | 説明 |
|---------|-------------|------|
| `/camera_01/color/image_raw` | `sensor_msgs/Image` | トップカメラからの入力画像 |

#### Published Topics
| Topic名 | メッセージ型 | 説明 |
|---------|-------------|------|
| `/top_camera/annotated_image` | `sensor_msgs/Image` | ランドマーク付きの画像 |
| `/top_camera/pose_landmarks` | `std_msgs/Float32MultiArray` | ポーズランドマーク座標（x,y,z） |
| `/top_camera/left_hand_landmarks` | `std_msgs/Float32MultiArray` | 左手ランドマーク座標（x,y,z） |
| `/top_camera/right_hand_landmarks` | `std_msgs/Float32MultiArray` | 右手ランドマーク座標（x,y,z） |

### chew_counter_node
#### Subscribed Topics
| Topic名 | メッセージ型 | 説明 |
|---------|-------------|------|
| `/front_camera/face_landmarks` | `std_msgs/Float32MultiArray` | 顔ランドマーク座標（Face Meshモデル、最大478ポイント、虹彩含む） |

#### Published Topics
| Topic名 | メッセージ型 | 説明 |
|---------|-------------|------|
| `/chewing/count` | `std_msgs/Int32` | 咀嚼の累積回数 |
| `/chewing/mar` | `std_msgs/Float32` | 平滑化後MAR |

## Parameter一覧

### front_camera_node

| Parameter名 | 型 | デフォルト値 | 説明 |
|-------------|----|-----------|----|
| `enable_roi` | bool | true | ROI（関心領域）を有効にするかどうか |
| `roi_x` | int | 0 | ROIの開始X座標 |
| `roi_y` | int | 0 | ROIの開始Y座標 |
| `roi_width` | int | 400 | ROIの幅 |
| `roi_height` | int | 300 | ROIの高さ |
| `min_detection_confidence` | double | 0.5 | 検出の最小信頼度 |
| `min_tracking_confidence` | double | 0.5 | 追跡の最小信頼度 |
| `enable_iris` | bool | true | 虹彩検出を有効にするかどうか |
| `refine_landmarks` | bool | true | 顔ランドマークの詳細化を有効にするかどうか |

### top_camera_node

| Parameter名 | 型 | デフォルト値 | 説明 |
|-------------|----|-----------|----|
| `enable_roi` | bool | true | ROI（関心領域）を有効にするかどうか |
| `roi_x` | int | 0 | ROIの開始X座標 |
| `roi_y` | int | 0 | ROIの開始Y座標 |
| `roi_width` | int | 400 | ROIの幅 |
| `roi_height` | int | 300 | ROIの高さ |
| `min_detection_confidence` | double | 0.5 | 検出の最小信頼度 |
| `min_tracking_confidence` | double | 0.5 | 追跡の最小信頼度 |

## Ref
[MediaPipe](https://chuoling.github.io/mediapipe/)