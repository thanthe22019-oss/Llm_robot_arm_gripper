# UR3 Perception

Package này biến ảnh RGB từ camera trên cao thành trạng thái môi trường có cấu
trúc. Vị trí động của block chỉ được suy ra từ ảnh; pose object trong scene YAML
chỉ được launch dùng để spawn mô phỏng.

## Interface

```text
/camera/image_raw          sensor_msgs/msg/Image (input)
/camera/detections_image   sensor_msgs/msg/Image (overlay)
/scene_state               std_msgs/msg/String (JSON, transient local)
/get_scene_state           std_srvs/srv/Trigger
```

`SceneState` chứa version, timestamp, năm object, occupancy của `zone_a`,
`zone_b`, `zone_c` và `temp_1`. Snapshot chỉ có `valid=true` khi nhìn thấy đủ
năm block và không có occupancy mơ hồ.

## Chạy

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur3_perception perception.launch.py
```

Kiểm tra trạng thái mới nhất:

```bash
ros2 service call /get_scene_state std_srvs/srv/Trigger '{}'
```

Scenario `blue_cube` chiếm `zone_b`:

```bash
ros2 launch ur3_perception perception.launch.py \
  scene_file:=$PWD/install/ur3_llm_control/share/ur3_llm_control/config/scene_blocked.yaml
```

Detector dùng HSV, lọc diện tích/hình dạng, chiếu tâm pixel qua homography rồi
phân loại vị trí theo biên zone. Ảnh overlay trong RViz hiển thị bounding box,
tên block, confidence và các vùng cố định.
