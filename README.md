# cibo
## 🛠️ Setup
1. Setup camera
Please follow link
[OrbbecSDK_ROS2](https://github.com/iHaruruki/OrbbecSDK_ROS2.git)

2. Setup python environment
Install python packages
```bash
pip3 install -U "numpy==1.26.4" "opencv-python==4.10.0.84"
pip3 install opencv-python mediapipe
```
Install ros packages
```bash
sudo apt install ros-humble-message-filters
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

## 🎮 How to use
### GitHubと同期
GitHubを更新している可能性があるので，ローカルリポジトリとリモートリポジトリを同期させる
```bash
cd ~/ros2_ws/src/cibo
```
```bash
git fetch
git switch main
git pull origin main
```
### Build
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select cibo
source install/setup.bash
```
### Run camera
```bash
ros2 launch orbbec_camera multi_camera.launch.py
```
- カメラの接続を確認する
```bash
ros2 launch cibo rviz.launch.py
```
rviz2の画面に`Top`&`Front`カメラの画像が表示されたら接続成功！

> [!TIP]
> カメラの接続に失敗した場合  
> [Multi-Camera](https://github.com/iHaruruki/OrbbecSDK_ROS2?tab=readme-ov-file#multi-camera)


To get the usb_port of the camera, plug in the camera and run the following command in the terminal:
```bash
ros2 run orbbec_camera list_devices_node
```
Result
```bash
ros2 run orbbec_camera list_devices_node 
[10/14 19:22:44.942823][info][29184][Context.cpp:68] Context created with config: default config!
[10/14 19:22:44.942840][info][29184][Context.cpp:73] Work directory=/home/######/ros2_ws, SDK version=v1.10.22-20250410-46139de
[10/14 19:22:44.942861][info][29184][LinuxPal.cpp:32] createObPal: create LinuxPal!
[10/14 19:22:45.314979][warning][29184][OpenNIDeviceInfo.cpp:186] New openni device matched.
[10/14 19:22:45.315007][warning][29184][OpenNIDeviceInfo.cpp:186] New openni device matched.
[10/14 19:22:45.315468][info][29184][LinuxPal.cpp:166] Create PollingDeviceWatcher!
[10/14 19:22:45.315525][info][29184][DeviceManager.cpp:15] Current found device(s): (2)
[10/14 19:22:45.315540][info][29184][DeviceManager.cpp:24] 	- Name: SV1301S_U3, PID: 0x0614, SN/ID: , Connection: USB3.0
[10/14 19:22:45.315553][info][29184][DeviceManager.cpp:24] 	- Name: SV1301S_U3, PID: 0x0614, SN/ID: , Connection: USB3.0
[INFO] [1760437365.352983950] [list_device_node]: serial: AY2T1120132
[INFO] [1760437365.353010630] [list_device_node]: usb port: 6-1.1.2 # Check
[INFO] [1760437365.379831358] [list_device_node]: serial: AY2T1120232
[INFO] [1760437365.379842742] [list_device_node]: usb port: 6-1.2.2 # Check
```
`ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/multi_camera.launch.py`を書き換える  
```python
`multi_camera.launch.py`
***
def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')
    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera_01',
            'usb_port': '2-1.1', # front_camera usb_port
            'device_num': '2',
            'sync_mode': 'standalone'
        }.items()
    )

    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera_02',
            'usb_port': '2-1.2.1', # top_camera usb_port
            'device_num': '2',
            'sync_mode': 'standalone'
        }.items()
    )
***
```


### cibo起動
```bash
ros2 launch cibo cibo_depth.launch.py
```
- ROI選択方法（骨格推定を行う範囲を指定する）
1. ノード起動後、OpenCVウィンドウが表示されます
2. マウスをドラッグして長方形のROIを選択します
3. ドラッグ中は青い矩形が表示され，確定後は緑の矩形で表示されます

### 出力画像を見る(OpenCV Image Show) 
```bash
ros2 run cibo image_show_node
```
### Run chewing count node / 咀嚼回数をカウントするNode
```bash
ros2 run cibo chew_counter_node
```
> [!WARNING]
> `ros2 run cibo chew_counter_node`  
> 調整中のため，正常に動作しない可能性があります．

### rosbag
画像を録画したい場合は，rosbagを利用
```bash
cd ~/ros2_ws/rosbag
# もし作成して場合は mkdir -p ~/ros2_ws/rosbag
```
Recode all topic
```bash
ros2 bag record -a
# This command is mode that record all topic.
```
Recode only specific topics
```bash
# ros2 bag record --topics <topic_name_1> <topic_name_2> <topic_name_3>
ros2 bag record --topics /camera_01/color/image_raw /camera_01/depth/image_raw /camera_02/color/image_raw /camera_02/depth/image_raw
```
Recode 
> [!WARNING]
> データサイズが大きいため，ストレージの空き容量に注意！


## 🚀 Node List

### front_camera_node
- **説明**: フロントカメラ用の骨格推定ノード.Face Meshモデルによる詳細な顔解析を実行

### top_camera_node  
- **説明**: トップカメラ用の骨格推定ノード.ポーズと手の検出に特化

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

式：
```math
$$ MAR = \frac{vertical distance (upper–lower lip)}{horizontal distance (left–right corner)} $$
```

MediaPipe Face Mesh の代表点で書くと：
```math
$$ MAR = \frac{|P_{13} - P_{14}|}{|P_{61} - P_{291}|} $$
```

## 🧩 Topic List

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

## 📦 Parameter List

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

## 👤 Authors

- **[iHaruruki](https://github.com/iHaruruki)** — Main author & maintainer

## 📚 Reference
[MediaPipe](https://chuoling.github.io/mediapipe/)