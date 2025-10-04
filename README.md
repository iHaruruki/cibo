# cibo
## Setup
1. Setup camera
Please follow link
[OrbbecSDK_ROS2](https://github.com/iHaruruki/OrbbecSDK_ROS2.git)

2. Setup python environment
If you are using a virtual environment, please refer to the following:
```bash
sudo apt install python3.10-venv
```
Create a python virtual environment
```bash
cd ros2_ws/src/cibo/
python3 -m venv cibo_ws
```
Install pyrhon packages
```bash
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
# Start a virtual environment if you want to use.
cd ros2_ws/src/cibo/
source ./cibo_ws/bin/activate
# Launch camera
ros2 launch orbbec_camera multi_camera.launch.py
```
- Run body_face_hand node
```bash
# Start a virtual environment if you want to use.
cd ros2_ws/src/cibo/
source ./cibo_ws/bin/activate
# Run node
ros2 run cibo body_face_hand_node
```
- rosbag
If you want to record images, use rosbag.
```bash
ros2 bag record -a
```
This command is mode that record all topic.