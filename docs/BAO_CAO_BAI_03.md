# BÁO CÁO BÀI THỰC HÀNH 03

## LLM Skill Planning với Gripper và Camera trên UR3e

- **Sinh viên:** Alex
- **Mã sinh viên:** 23020730
- **Video demo:** `[CHÈN LINK VIDEO SAU]`
- **GitHub repository:** [https://github.com/thanthe22019-oss/Llm_robot_arm_gripper](https://github.com/thanthe22019-oss/Llm_robot_arm_gripper)

> Các vị trí đánh dấu `[CHÈN ẢNH ...]` là placeholder. Thay bằng ảnh theo
> hướng dẫn tại `docs/HUONG_DAN_DEMO_VA_CHUP_ANH.md` trước khi xuất PDF.

## 1. Mục tiêu

Bài thực hành xây dựng hệ thống UR3e nhận câu lệnh ngôn ngữ tự nhiên, dùng LLM
để chọn và sắp xếp robot skill, kiểm tra kế hoạch rồi thực thi qua MoveIt 2.
Robot có gripper vật lý để gắp/thả vật trong Gazebo và camera trên cao để xác
định vị trí năm block cũng như trạng thái của ba vùng đích.

Kịch bản chính yêu cầu đặt `red_cube` vào `zone_b` khi `blue_cube` đang chiếm
vùng này. Hệ thống phải quan sát môi trường, dọn vật cản sang `temp_1`, thực
hiện yêu cầu chính và dùng camera xác minh kết quả.

## 2. Cấu trúc chương trình

```text
UR3e_LLM_Gripper_Camera_Bai03/
├── src/
│   ├── ur_simulation_gz/          Gazebo, ros2_control và MoveIt launch
│   ├── ur3_workcell_description/  UR3e, Robotiq 2F-85, camera, controller
│   ├── ur3_perception/            Nhận dạng màu và SceneState từ camera
│   ├── ur3_llm_control/           LLM planner, schema, validator, SceneGuard
│   └── ur3_robot_skills/          Skill server và executor
├── scripts/
│   ├── build_humble.sh
│   └── run_demo.sh
└── docs/
    ├── BAO_CAO_BAI_03.md
    ├── HUONG_DAN_DEMO_VA_CHUP_ANH.md
    └── evidence/
        ├── milestone7_runtime.txt
        ├── yellow_to_zone_b_runtime.txt
        ├── blue_to_zone_c_runtime.txt
        └── green_to_zone_a_runtime.txt
```

Các file YAML của scene chỉ cung cấp ground truth để Gazebo spawn model. Node
planner và robot skill không lấy trực tiếp pose spawn để quyết định vị trí gắp;
pose object và occupancy được suy ra từ ảnh camera.

## 3. Các package và node chính

| Package | Node/thành phần | Chức năng |
|---|---|---|
| `ur_simulation_gz` | Gazebo + `ros2_control` | Mô phỏng động lực học UR3e, controller tay máy và gripper |
| `ur3_workcell_description` | URDF/Xacro, SRDF | Mô tả UR3e, Robotiq 2F-85, camera, collision và MoveIt group |
| `ur3_perception` | `block_detector` | Nhận ảnh, tách màu, định vị block, tạo overlay và SceneState |
| `ur3_llm_control` | planner/validator | Biến câu lệnh thành plan JSON và kiểm tra plan theo trạng thái camera |
| `ur3_robot_skills` | `robot_skill_server` | Lập kế hoạch MoveIt và thực thi các skill mức cao |
| `ur3_robot_skills` | `execute_plan` | Điều phối camera → planner → validator → action server |

Camera publish `/camera/image_raw` và `/camera/camera_info`. Perception publish
`/camera/detections_image`, `/scene_state` và service `/get_scene_state`. Robot
skill được gọi qua action `/execute_skill`.

## 4. Môi trường mô phỏng, gripper và camera

Scene gồm UR3e, bàn thao tác, ba vùng `zone_a`, `zone_b`, `zone_c`, vị trí tạm
`temp_1` và năm block đỏ, vàng, xanh dương, xanh lá, tím. Hai scenario được
cung cấp: `scene_clear.yaml` và `scene_blocked.yaml`.

Gripper Robotiq 2F-85 có link, joint, collision, inertia và controller thật.
Chuỗi pick/place điều khiển ngón kẹp trong Gazebo. DetachableJoint chỉ khóa sau
khi ngón đã đóng và nhả trước khi mở nhằm làm grasp ổn định trong Gazebo
Fortress. Đường thực thi không gọi `/set_pose` để di chuyển object.

Camera RGB 640×480 đặt cố định trên bàn, chạy khoảng 15 Hz. Homography biến
tâm pixel sang tọa độ mặt bàn. Sai số đo tại năm block nằm trong khoảng
3,8–7,4 mm, thấp hơn ngưỡng 10 mm. Phần tô của ba zone dùng màu xám trung tính
và nhãn riêng. Cách này giữ nguyên tọa độ zone nhưng ngăn một cube cùng màu với
zone nhập thành một contour HSV, ví dụ `yellow_cube` nằm trong `zone_b`.

**[CHÈN ẢNH 01: Môi trường ban đầu — `images/01_moi_truong_ban_dau.png`]**

**[CHÈN ẢNH 02: Camera và SceneState — `images/02_camera_scene_state_ban_dau.png`]**

## 5. Các robot skill

| Skill | Tham số | Ý nghĩa |
|---|---|---|
| `observe_scene` | không | Lấy snapshot camera mới nhất |
| `check_zone` | `zone` | Kiểm tra vùng trống hay đang bị object chiếm |
| `pick` | `object` | Lấy pose camera, tiếp cận, đóng gripper và nâng vật |
| `place` | `object`, `zone` | Đặt object vào zone và xác minh hậu điều kiện |
| `place_temp` | `object`, `slot` | Dọn object sang vị trí tạm được camera xác nhận trống |
| `home` | không | Đưa robot về cấu hình an toàn và giải phóng góc nhìn camera |

Sau mỗi lần thả, robot về home rồi camera xác minh vị trí mới. Nếu planning,
grasp hoặc hậu điều kiện thất bại, skill trả mã lỗi và executor dừng. Quỹ đạo
Cartesian được kiểm tra tỷ lệ hoàn thành, bước khớp và tổng hành trình khớp;
khi cần, cùng pose đích được lập kế hoạch lại bằng OMPL. Các pose nghiêng theo
từng object giữ đầu ngón kẹp cách mặt bàn nhưng không thay đổi tọa độ cube hay
zone. `green_cube` dùng góc nghiêng 20° để tránh đầu ngón trái chạm bàn;
`purple_cube` dùng góc nghiêng 40° và grasp cao hơn 22 mm vì nằm ở mép xa vùng
làm việc.

## 6. Luồng câu lệnh đến robot

```text
Câu lệnh tiếng Việt/Anh
        ↓
Camera SceneState + scene_version
        ↓
LLM Planner (chỉ chọn skill và tham số)
        ↓
Structured Plan JSON
        ↓
State-aware Plan Validator
        ↓
SceneGuard kiểm tra lại trước từng skill
        ↓
Robot Skill Action Server
        ↓
MoveIt 2: IK, collision checking, trajectory
        ↓
ros2_control → UR3e + gripper trong Gazebo
        ↓
Camera xác minh hậu điều kiện
```

LLM không sinh joint angle, joint trajectory, velocity, effort hoặc torque.
Validator chỉ chấp nhận vocabulary đóng, object/zone hợp lệ, scene version còn
đúng, object nhìn thấy với confidence đủ cao và chuỗi hành động không mâu
thuẫn. SceneGuard dừng hoặc replan tối đa một lần nếu môi trường thay đổi ngoài
dự kiến trước khi robot cầm vật.

## 7. Logic xử lý zone bị chiếm

Camera ban đầu xác định `blue_cube` ở `zone_b`, `red_cube` ở trên bàn và
`temp_1` trống. Với câu lệnh “Đưa khối đỏ sang vùng B.”, plan hợp lệ là:

```json
{
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

Nếu không còn temp slot trống, camera stale, object bị mất hoặc confidence thấp,
validator từ chối kế hoạch trước khi action server nhận lệnh.

**[CHÈN ẢNH 03: LLM plan và validation — `images/03_llm_plan_validation.png`]**

**[CHÈN ẢNH 04: Dọn blue cube — `images/04_blue_cube_den_temp_1.png`]**

## 8. Cá nhân hóa theo MSSV

Cấu hình sinh viên dùng MSSV `23020730`. Với quy tắc của Bài 02,
`P = 30 mod 6 = 0`, nên ánh xạ ba vật chính là:

| Zone | Object |
|---|---|
| `zone_a` | `red_cube` |
| `zone_b` | `yellow_cube` |
| `zone_c` | `blue_cube` |

Bài 03 giữ cấu hình này để kiểm thử planner theo MSSV, đồng thời bổ sung kịch
bản bắt buộc có zone bị chiếm. `green_cube` và `purple_cube` được camera nhận
dạng và có thể được dùng làm object trong câu lệnh đơn.

## 9. Kết quả chạy thử

Kịch bản bắt buộc `red_cube → zone_b` đã hoàn thành đủ bảy skill. Camera ban
đầu phát hiện `blue_cube` đang chiếm `zone_b`; hệ thống chuyển khối này sang
`temp_1`, về home, sau đó gắp và đặt `red_cube` vào `zone_b`. Camera cuối xác
nhận `blue_cube` ở `temp_1` với confidence 0,9016 và `red_cube` ở `zone_b` với
confidence 0,8809. Terminal kết thúc bằng `TASK SUCCESS`.

Các trường hợp bổ sung đã chạy trong Gazebo:

| Trường hợp | Kết quả quan sát |
|---|---|
| `yellow_cube → zone_b` | `TASK SUCCESS`, camera xác nhận khối vàng ở `zone_b` |
| `blue_cube → zone_c` | `TASK SUCCESS`, camera xác nhận khối xanh dương ở `zone_c` |
| `green_cube → zone_a` | `TASK SUCCESS`, camera xác nhận `zone_a`, confidence 0,8657 |
| `purple_cube → zone_a` | Gắp, vận chuyển và thả thành công bằng pose nghiêng 40° |
| `purple_cube → zone_a`, sau đó `red_cube → zone_a` | Hệ thống dọn khối tím sang vị trí tạm rồi đặt khối đỏ vào A |

Trong lần chạy đầu của khối vàng, Gazebo cho thấy vật đã được đặt đúng nhưng
camera không còn tách được contour vì cube và zone cùng màu. Ba zone sau đó
được đổi sang màu trung tính, giữ nguyên tọa độ, và camera xác nhận hậu điều
kiện đúng. Với khối xanh lá, pose gắp thẳng đứng làm đầu ngón trái chạm bàn;
sau khi đổi riêng pose tay máy sang góc nghiêng 20°, toàn bộ chuỗi pick/place
hoàn thành.

Các log runtime lưu trong `docs/evidence/` sử dụng planner `mock` để kết quả có
thể lặp lại mà không phụ thuộc quota API. Đường thực thi sau planner vẫn giống
Gemini: SceneState → validator → SceneGuard → robot skill → MoveIt → Gazebo →
camera xác minh. Khi dùng `--planner gemini`, chỉ thành phần sinh structured
plan thay đổi; Gemini không sinh joint trajectory.

**[CHÈN ẢNH 05: SceneState cuối — `images/05_scene_state_ket_qua.png`]**

**[CHÈN ẢNH 06: TASK SUCCESS — `images/06_task_success.png`]**

**[CHÈN ẢNH 07: Tổng hợp ba ca còn lại — `images/07_kiem_thu_ba_khoi_con_lai.png`]**

**[CHÈN ẢNH 08: Pose nghiêng khi gắp xanh lá — `images/08_pose_nghieng_green_cube.png`]**

**[CHÈN ẢNH 09: Camera xác nhận xanh lá ở A — `images/09_scene_state_green_zone_a.png`]**

## 10. Hướng dẫn chạy

### 10.1. Build và chạy demo tổng

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
mkdir -p third_party
vcs import third_party < dependencies.repos
./scripts/build_humble.sh
./scripts/run_demo.sh
```

`run_demo.sh` mở Gazebo, RViz, camera, MoveIt và skill server; sau đó chạy tình
huống `blue_cube` đang chiếm `zone_b`. Planner mặc định là `mock` để demo không
phụ thuộc mạng. Kết quả cuối cần có `VALIDATION: SUCCESS` và `TASK SUCCESS`.

### 10.2. Chạy scene trống và gửi từng câu lệnh

Terminal 1:

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur3_robot_skills robot_skills.launch.py \
  run_demo:=false \
  scene_file:=$PWD/install/ur3_llm_control/share/ur3_llm_control/config/scene_clear.yaml
```

Terminal 2:

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ur3_robot_skills execute_plan \
  --command 'Đặt khối vàng vào ô B.' \
  --planner mock \
  --step-timeout-seconds 300
```

### 10.3. Ra lệnh cho Gemini

Nhập API key theo chế độ ẩn:

```bash
unset GEMINI_API_KEY
read -rsp "Nhập Gemini API key: " GEMINI_API_KEY
echo
export GEMINI_API_KEY
export GEMINI_MODEL="gemini-3.5-flash-lite"
```

Chạy toàn bộ tình huống zone bị chiếm bằng Gemini:

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
PLANNER=gemini \
COMMAND='Đưa khối đỏ sang vùng B.' \
./scripts/run_demo.sh
```

Hoặc giữ scene đang chạy và gửi từng lệnh từ terminal khác:

```bash
ros2 run ur3_robot_skills execute_plan \
  --command 'Đặt khối tím vào ô A.' \
  --planner gemini \
  --step-timeout-seconds 300

ros2 run ur3_robot_skills execute_plan \
  --command 'Đặt khối đỏ vào ô A.' \
  --planner gemini \
  --step-timeout-seconds 300
```

Ở lệnh thứ hai, camera thấy `zone_a` đang có khối tím. Gemini phải tạo kế hoạch
dọn khối tím tới `temp_1`, sau đó mới đặt khối đỏ vào `zone_a` và đưa robot về
home.

## 11. Liên kết sản phẩm

- Video demo: **[CHÈN LINK VIDEO SAU]**
- GitHub Public: [https://github.com/thanthe22019-oss/Llm_robot_arm_gripper](https://github.com/thanthe22019-oss/Llm_robot_arm_gripper)
- Log demo bắt buộc: `docs/evidence/milestone7_runtime.txt`
- Log vàng → B: `docs/evidence/yellow_to_zone_b_runtime.txt`
- Log xanh dương → C: `docs/evidence/blue_to_zone_c_runtime.txt`
- Log xanh lá → A: `docs/evidence/green_to_zone_a_runtime.txt`
- SceneState xanh lá → A: `docs/evidence/green_to_zone_a_scene_state.txt`
