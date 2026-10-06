# UR3e LLM Skill Planning — Bài thực hành 03

**GitHub:** [https://github.com/thanthe22019-oss/Llm_robot_arm_gripper](https://github.com/thanthe22019-oss/Llm_robot_arm_gripper)

Workspace riêng cho Bài thực hành 03, phát triển từ phần đã kiểm thử của Bài 02:

```text
Natural Language Command
  -> LLM Planner
  -> Structured Plan + Validator
  -> Robot Skills
  -> MoveIt 2
  -> UR3e + Physical Gripper
```

## Trạng thái

Milestone 1 đến Milestone 7 và phần kỹ thuật của Milestone 8 đã hoàn thành:

- UR3e, bàn, năm block và ba zone được dựng trong Gazebo/MoveIt;
- có một vị trí tạm an toàn `temp_1`;
- có scenario vùng đích trống và scenario `blue_cube` chiếm `zone_b`;
- giữ nguyên tọa độ ba cặp cube/zone từ Bài 02;
- tích hợp model chính thức Robotiq 2F-85 vào UR3e;
- có `GripperCommand` controller, MoveIt gripper group, end-effector và touch links;
- `pick/place` điều khiển ngón kẹp thật trong Gazebo, không gọi `/set_pose`;
- grasp được ổn định bằng DetachableJoint sau khi ngón kẹp đã đóng và được nhả
  trước khi mở gripper;
- demo độc lập đã gắp red cube, nâng 11 cm, di chuyển ngang, thả vào `zone_a`
  và giữ pose ổn định sau 5 giây;
- camera RGB cố định phía trên bàn publish ảnh 640×480 ở khoảng 15 Hz;
- `/camera/image_raw`, `/camera/camera_info` và TF camera đã được bridge sang ROS 2;
- ảnh camera nhìn thấy đủ năm block và toàn bộ vùng thao tác;
- phép đổi pixel sang mặt phẳng bàn có sai số đo được 3,8–7,4 mm tại năm block;
- detector HSV nhận đúng năm block từ ảnh, publish ảnh overlay và SceneState;
- camera xác định đúng `blue_cube` chiếm `zone_b` trong scenario blocked;
- khi di chuyển block trong Gazebo, SceneState cập nhật mà không sửa YAML;
- robot skill lấy pose gắp từ camera, kiểm tra confidence và xác minh lại vị
  trí sau khi thả;
- có `observe_scene`, `check_zone`, `place_temp` và tự về `home` trước khi
  camera kiểm tra hậu điều kiện;
- kịch bản khó `blue_cube: zone_b -> temp_1` đã chạy vật lý thành công và
  camera xác nhận `zone_b` trống;
- state-aware planner tự dọn object đang chiếm zone sang temp slot trống;
- validator mô phỏng occupancy, kiểm tra SceneState version, visibility,
  confidence, held object, zone trống và goal cuối;
- Gemini/OpenAI/mock planner nhận trực tiếp SceneState camera cùng câu lệnh;
- executor kiểm tra lại scene ngay trước từng skill và replan an toàn tối đa
  một lần khi state đổi;
- demo tiếng Việt đã dọn `blue_cube` sang `temp_1`, đặt `red_cube` vào
  `zone_b` và kết thúc `TASK SUCCESS`;
- 129 test Python/structural/Gazebo đều đạt.
- có script chạy demo một lệnh, log tham chiếu, báo cáo tiếng Việt và hướng
  dẫn chụp ảnh/quay video bàn giao.

Các tọa độ object trong file scene chỉ là ground truth để Gazebo spawn model.
`ur3_perception` không đưa các pose đó vào trạng thái quan sát; vị trí block và
occupancy đều được suy ra từ ảnh camera.

## Cấu trúc

```text
src/
├── ur_simulation_gz/     Gazebo Fortress và MoveIt 2 cho UR3e
├── ur3_llm_control/      Scenario, planner, validator và cấu hình MSSV
├── ur3_perception/       Nhận dạng block, overlay và SceneState từ camera
├── ur3_robot_skills/     MoveIt skills, gripper action server và executor
└── ur3_workcell_description/ UR3e + Robotiq 2F-85 cho Gazebo/MoveIt
third_party/              Dependency cục bộ, không đưa vào Git
scripts/build_humble.sh   Build bằng CMake hệ thống tương thích ROS Humble
scripts/run_demo.sh       Chạy toàn bộ kịch bản zone bị chiếm bằng một lệnh
docs/BAI_03_MILESTONES.md
```

## Chạy demo tổng bằng một lệnh

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
./scripts/run_demo.sh
```

Lệnh mặc định dùng mock planner để chạy offline, nhưng vẫn đi qua camera,
state-aware validator, MoveIt, gripper và Gazebo thật. Hướng dẫn chụp ảnh,
quay video và chạy với Gemini nằm tại
[`docs/HUONG_DAN_DEMO_VA_CHUP_ANH.md`](docs/HUONG_DAN_DEMO_VA_CHUP_ANH.md).
Bản báo cáo có sẵn placeholder ảnh và link tại
[`docs/BAO_CAO_BAI_03.md`](docs/BAO_CAO_BAI_03.md).

## Build

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
./scripts/build_humble.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Nên dùng script trên thay vì gọi `colcon build` trực tiếp. Máy hiện có CMake cài
bằng pip ở `~/.local/bin`; script ép dùng `/usr/bin/cmake` 3.22 để tương thích
ROS 2 Humble và tránh lỗi `FindPythonInterp.cmake`.

## Chạy scenario clear

Scenario mặc định có cả ba zone và `temp_1` đang trống:

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur3_llm_control llm_robot.launch.py
```

## Chạy scenario zone B bị chiếm

Trong scenario này, `blue_cube` nằm trong `zone_b`, còn `red_cube` nằm ngoài:

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur3_llm_control llm_robot.launch.py \
  scene_file:=$PWD/install/ur3_llm_control/share/ur3_llm_control/config/scene_blocked.yaml
```

Để kiểm tra không mở Gazebo GUI và RViz:

```bash
ros2 launch ur3_llm_control llm_robot.launch.py \
  gazebo_gui:=false \
  launch_rviz:=false \
  scene_file:=$PWD/install/ur3_llm_control/share/ur3_llm_control/config/scene_blocked.yaml
```

Nhấn `Ctrl+C` trong terminal chạy launch để dừng toàn bộ hệ thống.

## Kiểm tra camera và hiệu chuẩn — Milestone 3

Sau khi chạy `llm_robot.launch.py`, mở terminal khác và dùng:

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic hz /camera/image_raw
ros2 topic echo /camera/camera_info --once
ros2 run tf2_ros tf2_echo world camera_optical_frame
```

RViz đã có display `Overhead Camera` đọc `/camera/image_raw`. Có thể kiểm tra
phép đổi một tâm pixel sang mặt phẳng bàn bằng:

```bash
ros2 run ur3_llm_control pixel_to_table 185.184 322.388
```

Kết quả tương ứng với tâm `red_cube` là:

```text
x=0.200002 y=0.179996 z=0.160000
```

Thông số nội tại, extrinsic và homography nằm trong
`src/ur3_llm_control/config/camera_calibration.yaml`.

## Chạy perception và SceneState — Milestone 4

Lệnh tổng mở mô phỏng, camera bridge, detector và RViz:

```bash
ros2 launch ur3_perception perception.launch.py
```

Đọc snapshot camera mới nhất:

```bash
ros2 service call /get_scene_state std_srvs/srv/Trigger '{}'
```

Chạy tình huống `blue_cube` đang chiếm `zone_b`:

```bash
ros2 launch ur3_perception perception.launch.py \
  scene_file:=$PWD/install/ur3_llm_control/share/ur3_llm_control/config/scene_blocked.yaml
```

Kết quả chính cần thấy trong JSON:

```json
{
  "source": "camera",
  "valid": true,
  "objects": {"blue_cube": {"location": "zone_b"}},
  "zones": {"zone_b": {"occupied": true, "object": "blue_cube"}}
}
```

RViz đọc ảnh đã đánh dấu từ `/camera/detections_image`. Service trả thất bại
nếu chưa có ảnh, snapshot quá cũ, thiếu block hoặc trạng thái bị mơ hồ.

## Chạy skill dùng camera — Milestone 5

Khởi động scenario `zone_b` bị `blue_cube` chiếm, kèm MoveIt, gripper và
perception:

```bash
ros2 launch ur3_robot_skills robot_skills.launch.py \
  run_demo:=false \
  scene_file:=$PWD/install/ur3_llm_control/share/ur3_llm_control/config/scene_blocked.yaml
```

Trong terminal khác, dọn khối xanh sang vị trí tạm:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 action send_goal /execute_skill ur3_robot_skills/action/ExecuteSkill \
  "{skill: pick, object_name: blue_cube, zone_name: temp_1}" --feedback

ros2 action send_goal /execute_skill ur3_robot_skills/action/ExecuteSkill \
  "{skill: place_temp, object_name: blue_cube, zone_name: temp_1}" --feedback
```

Kết quả cuối phải có `SUCCESS`, thông báo camera đã xác nhận `blue_cube` trong
`temp_1`, còn `/get_scene_state` báo `zone_b.occupied=false`.

## State-aware Planner và Validator — Milestone 6

Kế hoạch mới chỉ dùng skill mức cao và gắn với đúng version camera:

```json
{
  "scene_version": 7,
  "goal": {"object": "red_cube", "destination": "zone_b"},
  "plan": [
    {"skill": "check_zone", "zone": "zone_b"},
    {"skill": "pick", "object": "blue_cube"},
    {"skill": "place_temp", "object": "blue_cube", "slot": "temp_1"},
    {"skill": "home"},
    {"skill": "pick", "object": "red_cube"},
    {"skill": "place", "object": "red_cube", "zone": "zone_b"},
    {"skill": "home"}
  ]
}
```

`validate_state_aware_plan()` từ chối plan nếu zone chưa được kiểm tra, zone
hoặc temp slot đang bị chiếm, object không thấy/confidence thấp, scene version
đã đổi, gripper giữ sai vật, plan không đạt goal hoặc chứa điều khiển joint.

## Chạy end-to-end — Milestone 7

Kiểm tra offline bằng mock planner, không cần API key:

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur3_robot_skills end_to_end.launch.py \
  planner:=mock \
  command:='Đưa khối đỏ sang vùng B.'
```

Launch mặc định dùng `scene_blocked.yaml`: camera thấy `blue_cube` đang chiếm
`zone_b`; planner phải sinh chuỗi dọn vật sang `temp_1` trước. Thành công khi
terminal in đủ 7 skill `SUCCESS` và dòng cuối là `TASK SUCCESS`.

Để dùng Gemini, nhập key ẩn rồi đổi planner:

```bash
read -rsp "Nhập Gemini API key: " GEMINI_API_KEY
echo
export GEMINI_API_KEY

ros2 launch ur3_robot_skills end_to_end.launch.py \
  planner:=gemini \
  command:='Đưa khối đỏ sang vùng B.'
```

Planner chỉ chọn skill và tham số. MoveIt vẫn tự lập trajectory, kiểm tra va
chạm và giới hạn khớp; LLM không được sinh joint trajectory.

## Chạy kiểm thử gripper vật lý — Milestone 2

Launch sau khởi động Gazebo, MoveIt, action server và tự chạy
`pick(red_cube) → place(red_cube, zone_a)`:

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur3_robot_skills robot_skills.launch.py
```

Thành công được xác nhận bằng dòng:

```text
MILESTONE 2 SUCCESS: red_cube was physically placed in zone_a
```

## Kết quả cần thấy

- Gazebo có `red_cube`, `yellow_cube`, `blue_cube`, `green_cube` và
  `purple_cube` trên bàn.
- RViz có nhãn `zone_a`, `zone_b`, `zone_c` và `temp_1` trên topic
  `/scene_markers`.
- RViz có ảnh từ camera trên cao; năm block đều nằm trong khung ảnh.
- Với `scene_blocked.yaml`, tâm `blue_cube` ở `(0.32, 0.06, 0.18)`, ngay trên
  `zone_b`; `red_cube` vẫn ở `(0.20, 0.18, 0.18)`.

## Chạy test

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
source /opt/ros/humble/setup.bash
PYTHONPATH=$PWD/src/ur3_llm_control:$PWD/src/ur3_perception:$PYTHONPATH \
  /usr/bin/python3 -m pytest -q \
    src/ur3_llm_control/test \
    src/ur3_perception/test \
    src/ur3_workcell_description/test \
    src/ur_simulation_gz/test
```

Kế hoạch đầy đủ nằm tại [docs/BAI_03_MILESTONES.md](docs/BAI_03_MILESTONES.md).
