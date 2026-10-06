# BÁO CÁO BÀI THỰC HÀNH 03

## LLM Skill Planning với Gripper và Camera trên UR3e

**Sinh viên:** Alex
**Mã sinh viên:** 23020730
**Video demo:** `[CHÈN LINK VIDEO SAU]`
**GitHub repository:** [https://github.com/thanthe22019-oss/Llm_robot_arm_gripper](https://github.com/thanthe22019-oss/Llm_robot_arm_gripper)

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
    ├── BAI_03_MILESTONES.md
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
zone.

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

## 9. Kết quả kiểm thử

Toàn bộ **129 test** Python, structural và Gazebo đã đạt. Hai test hồi quy mới
bảo đảm màu hiển thị zone có độ bão hòa thấp và pose xanh lá duy trì góc nghiêng
an toàn. Các nhóm kiểm thử bao gồm:

- camera geometry và perception năm màu;
- scene configuration và workcell description;
- schema, planner, MSSV và state-aware validator;
- stale scene, no-free-slot, wrong-held-object và direct-control rejection;
- SceneGuard và replan an toàn;
- controller, gripper, MoveIt launch và Gazebo integration.

Demo bắt buộc `red_cube → zone_b` hoàn thành đủ bảy skill. Camera cuối xác nhận
`blue_cube` ở `temp_1` với confidence 0,9016 và `red_cube` ở `zone_b` với
confidence 0,8809. Terminal kết thúc bằng `TASK SUCCESS`.

Ngoài kịch bản chính, ba cube còn lại đã được kiểm thử vật lý riêng trên
`scene_clear.yaml` bằng camera, MoveIt, gripper và Gazebo:

| Ca kiểm thử | Kết quả | Hậu điều kiện |
|---|---|---|
| `yellow_cube → zone_b` | `TASK SUCCESS` | camera xác nhận `yellow_cube.location=zone_b` |
| `blue_cube → zone_c` | `TASK SUCCESS` | camera xác nhận `blue_cube.location=zone_c` |
| `green_cube → zone_a` | `TASK SUCCESS` | camera xác nhận `green_cube.location=zone_a`, confidence 0,8657 |

Trong lần đầu của ca màu vàng, cube đã được đặt đúng theo pose Gazebo nhưng
camera mất contour vì cube và zone cùng màu. Sau khi đổi phần tô zone sang màu
trung tính, camera xác nhận được hậu điều kiện. Với xanh lá, pose gắp thẳng đứng
làm đầu ngón trái chạm bàn; pose tay máy được nghiêng 20° rồi ca kiểm thử chạy
thành công. Tọa độ ban đầu của cube và zone không thay đổi.

Các ca runtime ở trên dùng planner `mock` để kiểm thử lặp lại được và không phụ
thuộc API. Planner này vẫn nhận SceneState, sinh đúng structured plan và đi qua
cùng validator, SceneGuard, skill server, MoveIt và robot mô phỏng như planner
Gemini; nó không chứng minh một request Gemini trực tiếp trong các log này.

**[CHÈN ẢNH 05: SceneState cuối — `images/05_scene_state_ket_qua.png`]**

**[CHÈN ẢNH 06: TASK SUCCESS — `images/06_task_success.png`]**

**[CHÈN ẢNH 07: Tổng hợp ba ca còn lại — `images/07_kiem_thu_ba_khoi_con_lai.png`]**

**[CHÈN ẢNH 08: Pose nghiêng khi gắp xanh lá — `images/08_pose_nghieng_green_cube.png`]**

**[CHÈN ẢNH 09: Camera xác nhận xanh lá ở A — `images/09_scene_state_green_zone_a.png`]**

## 10. Lệnh chạy tổng

```bash
cd ~/Interaction/UR3e_LLM_Gripper_Camera_Bai03
./scripts/build_humble.sh
./scripts/run_demo.sh
```

Mock planner được dùng mặc định để demo offline. Để gọi Gemini thật:

```bash
read -rsp "Nhập Gemini API key: " GEMINI_API_KEY
echo
export GEMINI_API_KEY
PLANNER=gemini ./scripts/run_demo.sh
```

## 11. Liên kết sản phẩm

- Video demo: **[CHÈN LINK VIDEO SAU]**
- GitHub Public: [https://github.com/thanthe22019-oss/Llm_robot_arm_gripper](https://github.com/thanthe22019-oss/Llm_robot_arm_gripper)
- Log demo bắt buộc: `docs/evidence/milestone7_runtime.txt`
- Log vàng → B: `docs/evidence/yellow_to_zone_b_runtime.txt`
- Log xanh dương → C: `docs/evidence/blue_to_zone_c_runtime.txt`
- Log xanh lá → A: `docs/evidence/green_to_zone_a_runtime.txt`
- SceneState xanh lá → A: `docs/evidence/green_to_zone_a_scene_state.txt`
