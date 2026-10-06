# Hướng dẫn demo, chụp ảnh và quay video Bài 03

## 1. Chuẩn bị

Đóng các phiên Gazebo/RViz cũ trước khi quay để tránh dùng nhầm dữ liệu của
lần chạy trước:

```bash
pkill -INT -f 'rviz2|ign gazebo|gz sim|ruby.*ignition' || true
sleep 3
```

Build dự án nếu source vừa thay đổi:

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
./scripts/build_humble.sh
```

## 2. Chạy demo bằng một lệnh

Mock planner chạy hoàn toàn offline và cho kết quả lặp lại được:

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
./scripts/run_demo.sh 2>&1 | tee /tmp/bai03_demo.log
```

Script tự mở scenario `scene_blocked.yaml`, Gazebo, RViz, camera/perception,
MoveIt, robot skill server và executor. Câu lệnh mặc định là:

```text
Đưa khối đỏ sang vùng B.
```

Để demo LLM Gemini thật, nhập key ẩn rồi chạy:

```bash
read -rsp "Nhập Gemini API key: " GEMINI_API_KEY
echo
export GEMINI_API_KEY
PLANNER=gemini ./scripts/run_demo.sh 2>&1 | tee /tmp/bai03_gemini_demo.log
```

Không ghi API key vào source, báo cáo, video hoặc lịch sử lệnh. Dòng cảnh báo
về AFC của thư viện Gemini không phải lỗi nếu planner vẫn trả JSON hợp lệ.

## 3. Chuỗi hành vi cần quan sát

1. Camera thấy `blue_cube` đang ở `zone_b` và `temp_1` đang trống.
2. Planner sinh plan có `scene_version` tương ứng với camera.
3. Validator chấp nhận plan.
4. Robot gắp `blue_cube`, đặt vào `temp_1`, rồi về `home`.
5. Camera xác nhận `zone_b` đã trống.
6. Robot gắp `red_cube`, đặt vào `zone_b`, rồi về `home`.
7. Camera xác nhận `red_cube` ở `zone_b` và terminal in `TASK SUCCESS`.

Plan đúng phải có bảy bước:

```text
check_zone(zone_b)
pick(blue_cube)
place_temp(blue_cube, temp_1)
home()
pick(red_cube)
place(red_cube, zone_b)
home()
```

## 4. Danh sách ảnh cần chụp

Tạo thư mục lưu ảnh:

```bash
mkdir -p ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03/docs/images
```

Trên Ubuntu, nhấn `PrtSc` để chụp toàn màn hình hoặc `Shift+PrtSc` để chọn
vùng. Lưu ảnh theo đúng tên dưới đây để thay placeholder trong báo cáo.

### Ảnh 1 — Môi trường ban đầu

- Chụp cửa sổ Gazebo trước khi robot gắp vật.
- Khung hình cần thấy UR3e, gripper, bàn, năm block và ba zone.
- Cần nhìn rõ `blue_cube` nằm trong `zone_b`.
- Tên file: `01_moi_truong_ban_dau.png`.

Nếu robot bắt đầu quá nhanh, chạy riêng perception để giữ nguyên scene:

```bash
source /opt/ros/humble/setup.bash
source ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03/install/setup.bash
ros2 launch ur3_perception perception.launch.py \
  scene_file:=$HOME/Interaction/UR3e_LLM_Gripper_Camera_Bai03/install/ur3_llm_control/share/ur3_llm_control/config/scene_blocked.yaml
```

### Ảnh 2 — Camera nhận dạng và trạng thái zone

- Trong RViz, bật display `Overhead Camera`.
- Chụp ảnh overlay có bounding box và tên của năm block.
- Mở terminal khác và chạy lệnh dưới; chụp kèm phần JSON thể hiện
  `zone_b.occupied=true` và `object=blue_cube`:

```bash
ros2 service call /get_scene_state std_srvs/srv/Trigger '{}'
```

- Tên file: `02_camera_scene_state_ban_dau.png`.

### Ảnh 3 — Plan do LLM sinh và Validator

- Chụp terminal tại phần `CAMERA STATE`, `LLM PLAN` và
  `VALIDATION: SUCCESS`.
- Plan phải có bước dọn `blue_cube` sang `temp_1` trước khi gắp khối đỏ.
- Tên file: `03_llm_plan_validation.png`.

### Ảnh 4 — Gripper đang xử lý vật cản

- Chụp Gazebo khi gripper giữ `blue_cube` hoặc vừa đặt nó vào `temp_1`.
- Khung hình nên thấy hai ngón kẹp tiếp xúc với cube.
- Tên file: `04_blue_cube_den_temp_1.png`.

### Ảnh 5 — Trạng thái cuối từ camera

- Sau khi terminal báo `TASK SUCCESS`, chạy lại `/get_scene_state`.
- Chụp overlay/JSON thể hiện `blue_cube.location=temp_1` và
  `red_cube.location=zone_b`.
- Tên file: `05_scene_state_ket_qua.png`.

### Ảnh 6 — Kết quả thực thi

- Chụp terminal có các skill `SUCCESS` và dòng `TASK SUCCESS`.
- Tên file: `06_task_success.png`.

### Chuẩn bị cho ảnh kiểm thử các khối còn lại

Mở scene trống ở terminal thứ nhất. Lệnh này mở Gazebo và RViz nhưng không tự
chạy robot:

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur3_robot_skills robot_skills.launch.py \
  run_demo:=false \
  scene_file:=$PWD/install/ur3_llm_control/share/ur3_llm_control/config/scene_clear.yaml
```

Trong terminal thứ hai, source lại môi trường rồi chạy lần lượt ba ca. Chỉ chạy
lệnh tiếp theo sau khi lệnh trước in `TASK SUCCESS` và robot đã về home:

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run ur3_robot_skills execute_plan \
  --command 'Đặt khối vàng vào ô B.' \
  --planner mock --step-timeout-seconds 300 \
  2>&1 | tee docs/evidence/yellow_to_zone_b_runtime.txt

ros2 run ur3_robot_skills execute_plan \
  --command 'Đặt khối xanh dương vào ô C.' \
  --planner mock --step-timeout-seconds 300 \
  2>&1 | tee docs/evidence/blue_to_zone_c_runtime.txt

ros2 run ur3_robot_skills execute_plan \
  --command 'Đặt khối xanh lá vào ô A.' \
  --planner mock --step-timeout-seconds 300 \
  2>&1 | tee docs/evidence/green_to_zone_a_runtime.txt
```

### Ảnh 7 — Tổng hợp ba ca kiểm thử còn lại

- Sau khi chạy xong, mở terminal đủ rộng và lọc ba log:

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
grep -H -E "placed .*verified by camera|TASK SUCCESS" \
  docs/evidence/yellow_to_zone_b_runtime.txt \
  docs/evidence/blue_to_zone_c_runtime.txt \
  docs/evidence/green_to_zone_a_runtime.txt
```

- Ảnh phải thấy mỗi file có hậu điều kiện camera và `TASK SUCCESS`.
- Tên file: `07_kiem_thu_ba_khoi_con_lai.png`.

### Ảnh 8 — Pose nghiêng khi gắp khối xanh lá

- Nếu cần chụp lại, khởi động `scene_clear.yaml` từ đầu rồi chỉ chạy ca xanh lá.
- Chụp Gazebo lúc hai ngón kẹp vừa giữ `green_cube` hoặc đang rút lên.
- Lấy góc nhìn ngang để thấy đầu ngón không xuyên mặt bàn và cổ tay nghiêng
  khoảng 20°. Tọa độ cube và zone không bị dịch để tạo thuận lợi giả.
- Tên file: `08_pose_nghieng_green_cube.png`.

### Ảnh 9 — Camera xác nhận xanh lá ở zone A

- Sau khi ca xanh lá in `TASK SUCCESS`, chạy:

```bash
ros2 service call /get_scene_state std_srvs/srv/Trigger '{}'
```

- Chụp RViz overlay cùng terminal, hoặc chỉ chụp terminal đủ rõ để thấy:
  `green_cube.location=zone_a`, `zone_a.occupied=true` và
  `zone_a.object=green_cube`.
- Tên file: `09_scene_state_green_zone_a.png`.

## 5. Quay video demo

Video nên dài 3–6 phút và gồm các phần sau:

1. Giới thiệu MSSV `23020730`, kiến trúc và câu lệnh người dùng.
2. Cho thấy scene ban đầu và camera xác định `zone_b` bị chiếm.
3. Cho thấy JSON plan cùng kết quả validator.
4. Quay liên tục quá trình dọn `blue_cube`, gắp/thả `red_cube` và về home.
5. Cho thấy SceneState cuối và dòng `TASK SUCCESS`.
6. Nêu rõ LLM chỉ sinh skill-level plan; MoveIt sinh trajectory và kiểm tra
   va chạm; gripper vật lý thực hiện gắp/thả trong Gazebo.

Không cắt video tại những đoạn robot đang gắp hoặc thả. Việc quay liên tục
giúp chứng minh object không bị đổi pose trực tiếp.

## 6. Dừng sạch sau demo

Nhấn `Ctrl+C` tại terminal chạy script, đợi các process thoát rồi kiểm tra:

```bash
pgrep -af 'rviz2|ign gazebo|gz sim|move_group|robot_skill_server'
```

Nếu còn process của lần demo, dùng lệnh `pkill -INT` ở đầu tài liệu và kiểm tra
lại. Không mở phiên demo mới khi phiên trước vẫn còn chạy.

## 7. Kiểm tra log nhanh

```bash
grep -E 'CAMERA STATE|LLM PLAN|VALIDATION|SUCCESS|FAILED|TASK' \
  /tmp/bai03_demo.log
```

Các log tham chiếu đã kiểm thử nằm trong `docs/evidence/`, gồm demo bắt buộc
`milestone7_runtime.txt` và ba ca `yellow_to_zone_b_runtime.txt`,
`blue_to_zone_c_runtime.txt`, `green_to_zone_a_runtime.txt`.
