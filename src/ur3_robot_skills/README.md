# Kỹ năng robot UR3e — Milestone 2 và 5

Package này xây dựng bốn kỹ năng điều khiển trên MoveIt 2:

- `home()` đưa robot về cấu hình reset gập an toàn, tránh singularity;
- `move_above(object)` di chuyển `gripper_tcp` tới pose an toàn phía trên cube;
- `pick(object)` mở gripper vật lý, hạ theo Cartesian, đóng ngón, giữ cube và
  nâng lên;
- `place(object, zone)` nâng cube lên mặt phẳng chuyển tiếp an toàn, di chuyển tới
  vùng đích, hạ cube, mở ngón, detach và đồng bộ collision model MoveIt.
- `observe_scene()` kiểm tra snapshot camera mới nhất;
- `check_zone(zone)` đọc occupancy do camera suy ra;
- `place_temp(object, slot)` dọn object đang chiếm zone sang vị trí tạm.

`pick` lấy tọa độ object từ `/get_scene_state` và từ chối snapshot cũ, invalid,
mất object hoặc confidence dưới ngưỡng. `place`/`place_temp` đưa robot về `home`
sau khi nhả vật để giải phóng góc nhìn camera, rồi chỉ trả `SUCCESS` khi camera
xác nhận hậu điều kiện.

Gazebo không dùng `/set_pose` để kéo vật. Sau khi hai ngón đã đóng quanh cube,
DetachableJoint giữ ổn định tiếp xúc trong mô phỏng; joint được nhả trước khi
gripper mở để cube rơi và ổn định bằng vật lý Gazebo.

Các lệnh được nhận qua action `/execute_skill`, khai báo trong
`action/ExecuteSkill.action`. Kết quả gồm mã và tên trạng thái `SUCCESS`,
`FAILED`, `INVALID_OBJECT`, `INVALID_ZONE`, `PLANNING_FAILED`,
`EXECUTION_FAILED` hoặc `INVALID_SKILL`.

Quỹ đạo được kiểm tra giới hạn khớp, bước nhảy khớp, tổng góc di chuyển và va
chạm trước khi gửi tới `joint_trajectory_controller`. Nghiệm IK được quy đổi
về biểu diễn gần trạng thái khớp hiện tại; mỗi trajectory giới hạn tổng hành
trình 5.0 rad cho khớp wrist và 5.5 rad cho các khớp còn lại để loại nhánh gần
một vòng 360 độ. Skill `home`
dùng named state `test_configuration` của UR làm safe-home vì state `home` thẳng
mặc định có singularity và tạo quãng quay cổ tay lớn sau khi thả ở zone B. Các
chuyển động hạ/nâng dùng Cartesian path. Nếu Cartesian descent ở mặt phẳng chuyển tiếp
không khả thi, MoveIt replanning tới cùng pose rồi mới hạ tiếp. Nếu một bước
thất bại, executor không gửi bước kế tiếp.

Ba cube nằm trên hàng `x = 0.20 m` và đối diện zone cùng màu trên hàng
`x = 0.32 m`: đỏ/A cùng `y = 0.18 m`, vàng/B cùng `y = 0.06 m`, xanh/C cùng
`y = -0.06 m`. Các tọa độ này được giữ cố định. Khả năng tiếp cận được xử lý
bằng orientation gắp theo từng object, pose thả thẳng đứng và độ nâng chuyển
tiếp theo object trong `config/robot_skills.yaml`. Executor truyền nội bộ zone
của bước `place` kế tiếp vào `pick`; skill server thử các nghiệm IK và chọn nhánh
có Cartesian transfer an toàn tới đúng zone đó. JSON plan vẫn giữ dạng
`pick -> place -> home`.

## Biên dịch

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
source /opt/ros/humble/setup.bash
./scripts/build_humble.sh
source install/setup.bash
```

## Chạy toàn bộ

```bash
ros2 launch ur3_robot_skills robot_skills.launch.py
```

Launch file khởi động UR3e, Gazebo, MoveIt, RViz, scene, skill server và mặc
định chạy kiểm thử độc lập `pick(red_cube) → place(red_cube, zone_a)`.
Thành công được xác nhận bằng dòng:

```text
MILESTONE 2 SUCCESS: red_cube was physically placed in zone_a
```

Có thể chọn cube và zone khác ngay trên lệnh launch. Ví dụ chuyển
`yellow_cube` tới `zone_a`:

```bash
ros2 launch ur3_robot_skills robot_skills.launch.py \
  demo_object:=yellow_cube demo_zone:=zone_a
```

Để chỉ khởi động action server mà không chạy demo tự động, dùng launch
argument `run_demo:=false`.


## Chạy câu lệnh tự nhiên end-to-end

Tạo key tại [Google AI Studio](https://aistudio.google.com/app/apikey), sau đó
nhập key ẩn trong terminal và chạy launch tổng:

```bash
read -rsp "Nhập Gemini API key: " GEMINI_API_KEY
echo
export GEMINI_API_KEY

ros2 launch ur3_robot_skills end_to_end.launch.py \
  command:='Đưa khối đỏ sang vùng B.'
```

Gemini với model `gemini-3.5-flash-lite` là mặc định. Dùng `planner:=mock` để
kiểm tra offline hoặc `planner:=openai model:=gpt-4o-mini` nếu muốn dùng lại
OpenAI. Mọi chế độ đều sinh JSON, validate toàn bộ plan, rồi mới gọi action
`/execute_skill`.
