# cibo
## Setup
1. Setup camera
Please follow link
[OrbbecSDK_ROS2](https://github.com/iHaruruki/OrbbecSDK_ROS2.git)
2. Setup cibo Repositories
[cibo](https://github.com/iHaruruki/cibo.git)

---

## How to use
- Run camera
```bash
ros2 launch orbbec_camera multi_camera.launch.py
```
- Run body_face_hand
```bash
ros2 cibo run body_face_hand_node
```