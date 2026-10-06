# UR3e LLM Skill Planning với Gripper và Camera

Repository: [thanthe22019-oss/Llm_robot_arm_gripper](https://github.com/thanthe22019-oss/Llm_robot_arm_gripper)

Dự án mô phỏng UR3e trên ROS 2 Humble, MoveIt 2 và Gazebo. Người dùng có thể
ra lệnh bằng tiếng Việt hoặc tiếng Anh; Gemini chuyển câu lệnh thành chuỗi
robot skill, sau đó validator kiểm tra trước khi MoveIt lập trajectory và robot
thực thi bằng gripper vật lý trong Gazebo.

## 1. Chuẩn bị lần đầu

Yêu cầu máy đã cài ROS 2 Humble, MoveIt 2, Gazebo Fortress, `colcon` và `vcs`.

```bash
cd ~/Interaction
git clone https://github.com/thanthe22019-oss/Llm_robot_arm_gripper.git
cd Llm_robot_arm_gripper

mkdir -p third_party
vcs import third_party < dependencies.repos
python3 -m pip install --user google-genai

./scripts/build_humble.sh
```

Nếu đã clone repository và build thành công thì các lần sau không cần làm lại
phần này. Chỉ build lại khi source hoặc file cấu hình thay đổi.

## 2. Chạy toàn bộ demo bằng một lệnh

```bash
cd ~/Interaction/Llm_robot_arm_gripper
./scripts/run_demo.sh
```

Lệnh này tự mở Gazebo, RViz, camera, MoveIt, robot skill server và executor.
Scene mặc định là tình huống `blue_cube` đang chiếm `zone_b`. Planner offline
sẽ dọn khối xanh dương sang `temp_1`, đặt khối đỏ vào `zone_b`, rồi đưa robot
về home.

Kết quả cuối terminal cần có:

```text
VALIDATION: SUCCESS
...
TASK SUCCESS
```

Có thể chạy không mở giao diện để giảm tải máy:

```bash
GAZEBO_GUI=false LAUNCH_RVIZ=false ./scripts/run_demo.sh
```

## 3. Chạy hệ thống để gửi nhiều câu lệnh

### Terminal 1 — khởi động scene trống

Ba zone và `temp_1` ban đầu đều trống:

```bash
cd ~/Interaction/Llm_robot_arm_gripper
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur3_robot_skills robot_skills.launch.py \
  run_demo:=false \
  scene_file:=$PWD/install/ur3_llm_control/share/ur3_llm_control/config/scene_clear.yaml
```

Giữ terminal này chạy. Gazebo và RViz sẽ mở, camera bắt đầu cập nhật vị trí
các khối.

### Terminal 2 — gửi một câu lệnh offline

```bash
cd ~/Interaction/Llm_robot_arm_gripper
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run ur3_robot_skills execute_plan \
  --command 'Đặt khối vàng vào ô B.' \
  --planner mock \
  --step-timeout-seconds 300
```

Có thể thay nội dung sau `--command` bằng các câu khác, ví dụ:

```text
Đặt khối xanh dương vào ô C.
Đặt khối xanh lá vào ô A.
Đặt khối tím vào ô B.
Put the red cube in Zone A.
```

Hãy đợi lệnh hiện tại in `TASK SUCCESS` và robot về home rồi mới gửi lệnh kế
tiếp.

## 4. Chạy trường hợp zone đang bị chiếm

### Terminal 1 — khởi động scene có vật cản

Trong scene này, `blue_cube` nằm sẵn trong `zone_b`:

```bash
cd ~/Interaction/Llm_robot_arm_gripper
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur3_robot_skills robot_skills.launch.py \
  run_demo:=false \
  scene_file:=$PWD/install/ur3_llm_control/share/ur3_llm_control/config/scene_blocked.yaml
```

### Terminal 2 — yêu cầu đặt vật khác vào zone B

```bash
cd ~/Interaction/Llm_robot_arm_gripper
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run ur3_robot_skills execute_plan \
  --command 'Đưa khối đỏ sang vùng B.' \
  --planner mock \
  --step-timeout-seconds 300
```

Hệ thống sẽ dùng camera nhận ra `zone_b` bị chiếm, chuyển `blue_cube` sang
`temp_1`, sau đó mới đặt `red_cube` vào `zone_b`.

## 5. Ra lệnh trực tiếp cho Gemini điều khiển robot

Không ghi API key vào source hoặc README. Nhập key ẩn trong terminal:

```bash
unset GEMINI_API_KEY
read -rsp "Nhập Gemini API key: " GEMINI_API_KEY
echo
export GEMINI_API_KEY
export GEMINI_MODEL="gemini-3.5-flash-lite"
```

### Cách 1 — Gemini chạy toàn bộ demo bằng một lệnh

```bash
cd ~/Interaction/Llm_robot_arm_gripper
PLANNER=gemini \
COMMAND='Đưa khối đỏ sang vùng B.' \
./scripts/run_demo.sh
```

Script dùng `scene_blocked.yaml`, vì vậy Gemini phải sinh kế hoạch dọn vật đang
chiếm zone trước khi thực hiện yêu cầu chính.

### Cách 2 — gửi nhiều lệnh Gemini trong cùng một scene

Khởi động hệ thống ở Terminal 1 theo mục 3 hoặc mục 4. Trong Terminal 2, source
môi trường, nhập API key rồi gửi lệnh:

```bash
cd ~/Interaction/Llm_robot_arm_gripper
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run ur3_robot_skills execute_plan \
  --command 'Đặt khối tím vào ô A.' \
  --planner gemini \
  --step-timeout-seconds 300
```

Sau khi lệnh trên hoàn tất, có thể yêu cầu đặt khối đỏ vào chính ô A:

```bash
ros2 run ur3_robot_skills execute_plan \
  --command 'Đặt khối đỏ vào ô A.' \
  --planner gemini \
  --step-timeout-seconds 300
```

Camera sẽ cung cấp trạng thái scene hiện tại cho Gemini. Nếu `purple_cube` đang
chiếm `zone_a`, kế hoạch hợp lệ phải dọn khối tím tới vị trí tạm trước, rồi mới
gắp và đặt khối đỏ.

Một số câu lệnh Gemini có thể dùng:

```text
Đặt khối vàng vào ô B.
Di chuyển khối xanh dương sang vùng C.
Đưa khối tím ra vị trí tạm rồi đặt khối đỏ vào ô A.
Sắp xếp tất cả các khối theo mã sinh viên của tôi.
Put the green cube in Zone A.
```

LLM chỉ sinh tên skill, object, zone và thứ tự thực hiện. MoveIt 2 vẫn chịu
trách nhiệm tính IK, collision checking và joint trajectory.

## 6. Tên object và vị trí hợp lệ

Object:

```text
red_cube
yellow_cube
blue_cube
green_cube
purple_cube
```

Zone và vị trí tạm:

```text
zone_a
zone_b
zone_c
temp_1
```

Planner có thể hiểu các cách gọi tiếng Việt như “khối đỏ”, “khối vàng”, “khối
xanh dương”, “khối xanh lá”, “khối tím”, “ô A”, “vùng B” và “zone C”.

## 7. Xem trạng thái camera

Khi hệ thống đang chạy, mở terminal khác:

```bash
cd ~/Interaction/Llm_robot_arm_gripper
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 service call /get_scene_state std_srvs/srv/Trigger '{}'
```

Kết quả JSON cho biết block nào đang trên bàn, trong zone hoặc trong `temp_1`,
cùng confidence của camera.

## 8. Dừng hệ thống

Nhấn `Ctrl+C` tại terminal đang chạy launch. Nếu Gazebo hoặc RViz cũ vẫn còn:

```bash
pkill -INT -f 'rviz2|ign gazebo|gz sim|ruby.*ignition' || true
sleep 3
```

Sau đó mới mở phiên chạy mới để tránh nhận dữ liệu từ scene cũ.

## 9. Lỗi thường gặp

### Chưa build workspace

Nếu báo thiếu `install/setup.bash`:

```bash
cd ~/Interaction/Llm_robot_arm_gripper
./scripts/build_humble.sh
```

### Thiếu model Robotiq

```bash
cd ~/Interaction/Llm_robot_arm_gripper
mkdir -p third_party
vcs import third_party < dependencies.repos
./scripts/build_humble.sh
```

### Gemini báo thiếu thư viện

```bash
python3 -m pip install --user google-genai
```

### Gemini báo `401`

API key sai, hết hiệu lực hoặc chưa được export. Nhập lại bằng `read -rsp` như
mục 5.

### Gemini báo `429`

Project Gemini đang hết quota hoặc bị giới hạn request. Kiểm tra quota trong
Google AI Studio, hoặc chạy tạm với `--planner mock`.

### Robot dừng lâu ở một bước

Giữ `--step-timeout-seconds 300`, kiểm tra Gazebo không bị pause và không chạy
nhiều phiên Gazebo/RViz cùng lúc.
