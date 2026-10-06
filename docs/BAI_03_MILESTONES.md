# Kế hoạch triển khai Bài thực hành 03

## 1. Mục tiêu cuối

Xây dựng hệ thống UR3/UR3e trong Gazebo có gripper vật lý và camera, nhận câu
lệnh tự nhiên, quan sát trạng thái thật của năm block và ba zone, sinh plan có
cấu trúc, validate rồi thực thi bằng robot skill và MoveIt 2.

Kịch bản bắt buộc:

1. `blue_cube` đang chiếm `zone_b`.
2. Người dùng yêu cầu: `Put the red cube in Zone B.`
3. Camera xác nhận `zone_b` bị chiếm.
4. Hệ thống tìm một vị trí tạm còn trống.
5. Robot dùng gripper di chuyển `blue_cube` ra vị trí tạm.
6. Robot gắp `red_cube` và đặt vào `zone_b`.
7. Robot trở về `home`.

## 2. Đánh giá Bài 02 hiện tại

### Có thể tái sử dụng

- Hai package `ur3_llm_control` và `ur3_robot_skills`.
- Gemini/Mock/OpenAI planner và closed JSON schema.
- Plan Validator, Skill Executor và action `/execute_skill`.
- MoveIt Planning Scene, collision checking và Cartesian approach/lift.
- Cơ chế chọn nhánh IK, giới hạn quãng quay khớp và `home` giữa các nhiệm vụ.
- Launch Gazebo + MoveIt + RViz và hệ thống test hiện có.

### Chưa đáp ứng Bài 03

- `openGripper()` và `closeGripper()` mới là logical gripper, chỉ ghi log.
- Vật đang được giữ bằng MoveIt attach/detach và đồng bộ pose cuối qua
  `/world/.../set_pose`; chưa phải gripper vật lý giữ vật trong Gazebo.
- Scene mới có ba block; đề bài yêu cầu năm block.
- Vị trí block được đọc cố định từ `scene.yaml`; chưa có camera quan sát.
- Validator chỉ hiểu `pick`, `place`, `home`; chưa hiểu state của zone, vị trí
  tạm hoặc trường hợp zone bị chiếm.

## 3. Kiến trúc đích

```text
Camera Gazebo
    -> ros_gz_bridge
    -> block_detector
    -> scene_state_server
    -> SceneState JSON

Natural-language command + SceneState
    -> LLM Planner
    -> Structured Plan
    -> State-aware Plan Validator
    -> Skill Executor
    -> /execute_skill
    -> MoveIt 2 + Gripper Controller
    -> UR3e + physical gripper in Gazebo
    -> camera verification after each placement
```

Các package dự kiến:

```text
ur3_workcell_description/  Gripper, camera, URDF/Xacro, controllers, world
ur3_perception/            Camera detector và scene state service/topic
ur3_llm_control/           Prompt, planner, world-state validator
ur3_robot_skills/          Pick/place/temp/home và gripper execution
```

Có thể gộp `ur3_workcell_description` vào package hiện có nếu muốn ít package,
nhưng nên giữ `ur3_perception` riêng để tách perception khỏi planning.

## 4. Milestone 1 — Chốt baseline và dựng scene năm block

### Công việc

- Tạo branch phát triển Bài 03 từ commit Bài 02 đã chạy ổn định.
- Giữ toàn bộ regression test của Bài 02.
- Thêm `green_cube` và `purple_cube` vào scene.
- Khai báo ba zone và một vùng đặt tạm an toàn trên bàn.
- Tạo ít nhất hai cấu hình scenario:
  - `scene_clear.yaml`: zone đích trống;
  - `scene_blocked.yaml`: `blue_cube` nằm trong `zone_b`, `red_cube` nằm ngoài.
- Tọa độ trong YAML chỉ dùng để spawn ground-truth của Gazebo; planner không
  được đọc trực tiếp các tọa độ này làm trạng thái quan sát.

### Tiêu chí hoàn thành

- Gazebo và RViz hiển thị UR3e, bàn, năm block và ba zone.
- `scene_blocked.yaml` thể hiện rõ `zone_b` đã bị chiếm.
- Toàn bộ 88 test Bài 02 vẫn đạt.

### Trạng thái ngày 02/10/2026: hoàn thành

- [x] Tách workspace Bài 03 tại `~/Interaction/UR3e_LLM_Gripper_Camera_Bai03`.
- [x] Giữ nguyên tọa độ ba cube/zone đã kiểm thử từ Bài 02.
- [x] Thêm `green_cube`, `purple_cube` và `temp_1`.
- [x] Tạo `scene_clear.yaml` và `scene_blocked.yaml`.
- [x] Xác nhận Gazebo spawn đủ năm cube, ba zone và `temp_1`.
- [x] Xác nhận `blue_cube` có pose `(0.32, 0.06, 0.18)` trong `zone_b`.
- [x] Build thành công ba package bằng CMake hệ thống 3.22.1.
- [x] 88 test regression và 6 test Milestone 1 đều đạt (`94 passed`).

Chưa tạo commit theo yêu cầu; sẽ commit sau khi các phần cần thiết đã hoàn tất.

## 5. Milestone 2 — Tích hợp gripper vật lý

### Thiết kế đề xuất

Dùng gripper hai ngón song song, gắn vào flange/tool của UR3e. Gripper cần có:

- link, joint, collision và inertia đúng trong URDF/Xacro;
- hai finger pad có ma sát đủ để giữ cube;
- `ros2_control` controller, ưu tiên `GripperCommand` hoặc trajectory controller;
- MoveIt planning group `gripper`, end-effector và touch links;
- lệnh `open_gripper`/`close_gripper` có phản hồi thành công/thất bại.

MoveIt attach object vẫn có thể dùng để mô hình hóa collision của vật đang cầm,
nhưng không được dùng để kéo pose vật trong Gazebo. Chuyển động vật trong
Gazebo phải đến từ tiếp xúc của hai ngón gripper. Bỏ việc dùng `/set_pose` trong
chuỗi gắp/thả chính.

### Kiểm thử độc lập

1. Mở gripper.
2. Đưa gripper bao quanh một cube.
3. Đóng gripper tới lực/vị trí đặt trước.
4. Nâng cube ít nhất 0.10 m.
5. Giữ yên ít nhất 5 giây.
6. Di chuyển ngang rồi mở gripper.

### Tiêu chí hoàn thành

- [x] Model Robotiq 2F-85 chính thức có collision, inertia và ros2_control.
- [x] Có GripperCommand controller, MoveIt group, end-effector và touch links.
- [x] `pick/place` mở và đóng ngón kẹp thật, không dùng service đổi pose.
- [x] DetachableJoint chỉ khóa sau khi ngón đóng và nhả trước khi ngón mở để
  ổn định grasp trong Gazebo Fortress.
- [x] MoveIt nhận đúng collision geometry của gripper và vật đang cầm.
- [x] Demo red cube nâng 0.11 m, đi ngang và thả vào `zone_a`; pose Gazebo ổn
  định sau 5 giây tại x=0.3148, y=0.1793, z=0.1800 m.
- [x] Các đoạn hạ, nâng, chuyển ngang và retreat đều đạt Cartesian path 100%.
- [x] 100 test Python/structural/Gazebo đạt.

Milestone 2 hoàn thành. Không tạo commit theo yêu cầu; chuyển sang Milestone 3
để tích hợp camera.

## 6. Milestone 3 — Tích hợp camera và bridge dữ liệu

### Thiết kế đề xuất

- Camera RGB cố định phía trên bàn để giảm occlusion; có thể dùng RGB-D nếu
  package sẵn có ổn định.
- Khai báo `camera_link` và `camera_optical_frame`.
- Publish tối thiểu:
  - `/camera/image_raw`;
  - `/camera/camera_info`;
  - tùy chọn `/camera/depth/image_raw`.
- Bridge ảnh và camera info từ Gazebo sang ROS 2.
- Publish TF/extrinsic từ `world` hoặc `base_link` tới camera.
- Hiệu chuẩn phép biến đổi pixel `(u,v)` sang mặt phẳng bàn `(x,y)` bằng
  homography hoặc camera model + TF.

### Tiêu chí hoàn thành

- [x] `ros2 topic hz /camera/image_raw` cho dữ liệu ổn định.
- [x] RViz/ảnh ROS hiển thị đủ năm block và toàn bộ vùng thao tác.
- [x] Điểm chuẩn trên bàn chuyển từ pixel sang tọa độ robot với sai số không
  quá 1 cm.

### Trạng thái ngày 03/10/2026: hoàn thành

- [x] Thêm camera RGB cố định phía trên bàn, ảnh 640×480, 15 Hz, HFOV 60°.
- [x] Bridge `/camera/image_raw` và `/camera/camera_info` từ Gazebo sang ROS 2.
- [x] Publish đầy đủ chuỗi TF `world -> camera_link -> camera_optical_frame`
  và ánh xạ frame scoped của cảm biến Gazebo.
- [x] Thêm display `Overhead Camera` vào cấu hình RViz.
- [x] Lưu camera intrinsic, extrinsic và homography trong
  `camera_calibration.yaml`.
- [x] Thêm CLI `pixel_to_table` và unit test chuyển đổi hai chiều.
- [x] Đo trực tiếp trên ảnh Gazebo: sai số tâm của năm block từ 3,8 mm đến
  7,4 mm, nhỏ hơn mục tiêu 10 mm.
- [x] Camera publish ổn định khoảng 15,14 Hz và nhìn thấy đủ năm block.
- [x] Toàn bộ 105 test Python/structural/Gazebo đạt.

Milestone 3 hoàn thành. Không tạo commit theo yêu cầu; bước tiếp theo là
Milestone 4 — nhận dạng block và xây dựng SceneState từ camera.

## 7. Milestone 4 — Perception và SceneState từ camera

### Node dự kiến

- `block_detector`: HSV/color segmentation hoặc marker detection, trả tên,
  centroid pixel, pose trên bàn và confidence.
- `scene_state_server`: hợp nhất detection thành snapshot môi trường và xác định
  quan hệ `in_zone`, `on_table`, `at_temporary_position`.
- `perception_overlay`: publish ảnh có bounding box, tên object và zone để demo.

### SceneState đề xuất

```json
{
  "stamp": 123.45,
  "objects": {
    "red_cube": {"location": "table", "x": 0.20, "y": 0.18, "confidence": 0.98},
    "blue_cube": {"location": "zone_b", "x": 0.32, "y": 0.06, "confidence": 0.99}
  },
  "zones": {
    "zone_a": {"occupied": false, "object": null},
    "zone_b": {"occupied": true, "object": "blue_cube"},
    "zone_c": {"occupied": false, "object": null}
  }
}
```

Zone geometry có thể biết trước từ thiết kế bàn, nhưng object/occupancy bắt buộc
phải suy ra từ ảnh. Snapshot quá cũ, confidence thấp, object trùng lặp hoặc mất
khỏi camera phải làm hệ thống dừng, không đoán pose.

### Tiêu chí hoàn thành

- [x] Nhận dạng đúng cả năm màu ở nhiều vị trí trên bàn.
- [x] Xác định đúng zone trống và object đang chiếm zone.
- [x] Di chuyển một cube thủ công rồi perception cập nhật mà không sửa YAML.
- [x] Có unit test bằng ảnh mẫu và test sai số pixel-to-world.

### Trạng thái ngày 04/10/2026: hoàn thành

- [x] Tạo package `ur3_perception` và node `block_detector`.
- [x] Nhận diện năm màu bằng HSV, lọc diện tích/hình dạng và loại nhiễu lớn từ
  thân robot.
- [x] Tọa độ `(x, y)` của object chỉ lấy từ tâm pixel và homography; loader
  perception không chứa pose spawn của object.
- [x] Publish `/camera/detections_image` với bounding box, nhãn, confidence và
  biên các zone/temp slot.
- [x] Publish `/scene_state` dạng JSON và service `/get_scene_state`.
- [x] Snapshot có version, timestamp, freshness, trạng thái visible, location,
  confidence, zone occupancy và danh sách lỗi.
- [x] Scenario clear nhận đủ năm block và báo ba zone cùng `temp_1` đều trống.
- [x] Scenario blocked đo `blue_cube` tại `(0.321, 0.063, 0.180) m` và báo
  `zone_b` đang bị `blue_cube` chiếm.
- [x] Sau khi di chuyển thủ công `blue_cube` sang `temp_1`, camera đo
  `(0.445, 0.063, 0.180) m`, báo `zone_b` trống và `temp_1` bị chiếm mà không
  sửa YAML.
- [x] 5 unit test perception mới và toàn bộ 110 test regression đạt.

Milestone 4 hoàn thành. Không tạo commit theo yêu cầu; bước tiếp theo là
Milestone 5 — dùng SceneState camera làm nguồn pose cho robot skill và xác minh
post-condition sau gắp/thả.

## 8. Milestone 5 — Robot skill dùng perception và gripper

### Skill/API đề xuất

- `observe_scene()` hoặc service `get_scene_state()`.
- `check_zone(zone)`.
- `find_free_position()`.
- `open_gripper()` và `close_gripper()`.
- `pick(object)` dùng pose mới nhất từ camera.
- `place(object, zone)`.
- `place_temp(object, temp_slot)`.
- `home()`.

`find_free_position()` dùng một tập temp slot an toàn đã biết trên mặt bàn, sau
đó camera xác nhận slot nào đang trống. Đây là lựa chọn xác định và dễ validate
hơn việc để LLM tự sinh tọa độ Cartesian.

### Thay đổi skill server

- Chuyển từ pose object cố định sang pose do `scene_state_server` cung cấp.
- Kiểm tra freshness/confidence trước khi planning.
- Điều khiển gripper controller thật trong pick/place.
- Xác minh grasp bằng trạng thái finger/contact hoặc việc object còn đi theo
  sau khi nâng.
- Sau place, gọi camera xác nhận post-condition trước khi báo `SUCCESS`.

### Tiêu chí hoàn thành

- [x] Gắp được block từ pose camera thay vì pose object trong YAML.
- [x] Thả được vào zone trống và `temp_1` trống.
- [x] Nếu grasp, planning hoặc hậu điều kiện thất bại, skill trả lỗi và dừng.
- [x] Không có `/set_pose` trong đường thực thi gắp/thả.

### Trạng thái ngày 04/10/2026: hoàn thành

- [x] Skill server chờ `/get_scene_state`, kiểm tra nguồn camera, freshness,
  `valid`, visibility và confidence tối thiểu 0,70.
- [x] `pick()` đồng bộ pose đo từ camera sang Planning Scene trước khi lập kế
  hoạch; không đọc pose spawn của object làm pose gắp.
- [x] Thêm `observe_scene`, `check_zone` và `place_temp` vào action server.
- [x] Sau khi thả, robot tự về `home` để camera không bị cánh tay che, sau đó
  chỉ báo thành công khi camera xác nhận object ở đúng zone/slot.
- [x] Giữ nguyên tọa độ cube, zone và `temp_1`; xử lý giới hạn workspace bằng
  pose cổ tay nghiêng 20° và khoảng hở thả vật, đều qua MoveIt collision check.
- [x] Kịch bản blocked chạy thật thành công:
  `pick(blue_cube) -> place_temp(blue_cube, temp_1) -> home`.
- [x] Camera sau thao tác đo confidence 0,9037, báo `blue_cube.location=temp_1`,
  `temp_1.occupied=true` và `zone_b.occupied=false`.

Milestone 5 hoàn thành. Không tạo commit theo yêu cầu; chuyển sang Milestone 6.

## 9. Milestone 6 — State-aware Planner và Plan Validator

### Structured Plan đề xuất

Giữ JSON skill-level, không thêm joint/trajectory/pose tự do. Ví dụ:

```json
{
  "scene_version": 17,
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

### Luật Validator mới

- Chỉ chấp nhận năm object, ba zone và danh sách temp slot khai báo trước.
- Scene version của plan phải khớp snapshot perception hiện tại.
- Không được pick object không thấy hoặc confidence thấp.
- Không được place vào zone/slot đang bị chiếm.
- Nếu zone đích bị chiếm, occupant phải được di chuyển trước.
- `place_temp` phải dùng slot được camera xác nhận trống.
- Không được pick khi gripper đang giữ vật.
- `place` phải đúng object đang giữ.
- Plan phải đạt goal cuối và kết thúc bằng `home` với gripper rỗng.
- Cấm joint, trajectory, velocity, effort, torque, controller và tọa độ tự do.

Validator nên mô phỏng state sau từng bước để chứng minh plan không tự mâu
thuẫn. Nếu camera state thay đổi sau validation, executor hủy plan và yêu cầu
quan sát/replan.

### Tiêu chí hoàn thành

- [x] Plan hợp lệ cho zone trống được chấp nhận.
- [x] Plan đặt trực tiếp vào zone bị chiếm bị từ chối.
- [x] Plan di chuyển occupant sang temp rồi đặt object chính được chấp nhận.
- [x] Có test cho stale scene, no-free-slot, wrong-held-object và direct-control.

### Trạng thái ngày 04/10/2026: hoàn thành

- [x] Mở rộng closed skill vocabulary cho năm block, `observe_scene`,
  `check_zone` và `place_temp`.
- [x] Kế hoạch state-aware mang `scene_version` và goal có cấu trúc; version
  không khớp snapshot camera bị từ chối với `STALE_SCENE`.
- [x] Validator mô phỏng `held_object`, object location và occupancy sau từng
  bước để phát hiện place sai vật hoặc destination đang bị chiếm.
- [x] Bắt buộc kiểm tra zone đích, confidence tối thiểu, `home` giữa các cặp
  gắp/thả và `home` ở cuối plan.
- [x] Thêm deterministic state-aware planner: zone trống sinh chuỗi trực tiếp;
  zone bị chiếm tự dọn occupant sang temp slot trống trước.
- [x] Hết temp slot, mất object, confidence thấp, scene invalid/stale và trường
  điều khiển joint/trajectory đều dừng trước action server.
- [x] Toàn bộ 121 test Python/structural/Gazebo đạt.

Milestone 6 hoàn thành. Không tạo commit theo yêu cầu; bước tiếp theo là
Milestone 7 — đưa SceneState camera vào prompt/schema của Gemini và kiểm tra
lại snapshot ngay trước khi executor gửi từng skill.

## 10. Milestone 7 — Tích hợp LLM với trạng thái camera

### Công việc

- Gửi cho LLM câu lệnh, danh sách skill, SceneState rút gọn và các temp slot
  trống; không gửi ảnh thô nếu không cần.
- Cập nhật closed schema/prompt cho năm block và skill mới.
- Hỗ trợ tiếng Việt và tiếng Anh.
- Luôn chạy independent Validator sau Structured Output.
- Giữ mock planner xác định để test khi API/mạng lỗi.
- Trước mỗi pick/place, executor kiểm tra lại snapshot. Nếu state khác plan,
  dừng và replan tối đa một lần thay vì tiếp tục trên dữ liệu cũ.

### Câu kiểm thử

- `Put the red cube in Zone B.`
- `Đưa khối tím vào vùng A.`
- `Move the object occupying Zone C to a free temporary position.`
- Câu lệnh cố yêu cầu joint/trajectory để chứng minh hệ thống từ chối.

### Tiêu chí hoàn thành

- [x] LLM/mock planner sinh đúng plan khi zone trống.
- [x] LLM/mock planner sinh chuỗi relocation khi zone bị chiếm.
- [x] JSON sai hoặc hallucinated object/zone bị Validator chặn trước action server.

### Trạng thái ngày 04/10/2026: hoàn thành

- [x] Prompt và closed JSON schema nhận năm block, `scene_version`, goal,
  `check_zone`, `place_temp` và `temp_1`.
- [x] Gemini/OpenAI nhận câu lệnh cùng SceneState camera rút gọn; mock planner
  dùng đúng cùng state-aware contract để kiểm thử offline.
- [x] Hỗ trợ tên màu tiếng Việt/Anh cho đỏ, vàng, xanh dương, xanh lá và tím.
- [x] Executor đọc camera trước khi gọi planner và chạy independent validator
  sau output của planner.
- [x] Runtime `SceneGuard` kiểm tra lại object/occupancy ngay trước
  `check_zone`, `pick`, `place` và `place_temp`; scene đổi ngoài dự kiến làm
  dừng, chỉ replan tối đa một lần khi robot chưa giữ vật.
- [x] Launch tổng mặc định dùng `scene_blocked.yaml` và câu lệnh
  `Put the red cube in Zone B.`.
- [x] Chạy end-to-end bằng câu tiếng Việt `Đưa khối đỏ sang vùng B.` thành
  công đủ 7 bước và kết thúc bằng `TASK SUCCESS`.
- [x] Camera cuối cùng xác nhận `blue_cube` ở `temp_1` (confidence 0,9016) và
  `red_cube` ở `zone_b` (confidence 0,8809).
- [x] Toàn bộ 127 test đạt trong môi trường Gazebo sạch.

Milestone 7 hoàn thành. Không tạo commit theo yêu cầu; bước tiếp theo là
Milestone 8 — chuẩn hóa demo, ảnh/video, hướng dẫn bàn giao và báo cáo.

## 11. Milestone 8 — Demo bắt buộc, kiểm thử và bàn giao

### Kịch bản demo chính

Initial state từ camera:

```text
zone_b = occupied by blue_cube
red_cube = on table
at least one temporary slot = free
```

User command:

```text
Put the red cube in Zone B.
```

Chuỗi quan sát được trong log/video:

```text
CAMERA STATE
-> LLM PLAN
-> VALIDATION SUCCESS
-> pick(blue_cube)
-> place_temp(blue_cube, temp_1)
-> camera verification
-> pick(red_cube)
-> place(red_cube, zone_b)
-> camera verification
-> home()
-> TASK SUCCESS
```

### Kiểm thử cuối

- Unit test perception với ảnh mẫu.
- Unit test schema/validator/world-state transition.
- Integration test gripper open/close và grasp failure.
- Integration test zone trống.
- Integration test zone bị chiếm.
- Test không có temp slot: dừng an toàn.
- Test camera stale/mất object: dừng an toàn.
- Regression test Bài 02 và kiểm tra collision/giới hạn khớp.

### Sản phẩm bàn giao

- GitHub Public chứa source, launch, config, test và README tiếng Việt.
- Một lệnh launch tổng.
- Video demo có camera overlay, gripper thật và kịch bản zone bị chiếm.
- Báo cáo giải thích kiến trúc, perception, skill, validator và kết quả.
- Ảnh minh họa trạng thái camera trước/sau và log `TASK SUCCESS`.

### Trạng thái ngày 04/10/2026: phần kỹ thuật hoàn thành

- [x] Có một lệnh `./scripts/run_demo.sh` mở toàn bộ scenario bị chiếm và chạy
  chuỗi camera → planner → validator → skill → MoveIt → robot.
- [x] Kịch bản bắt buộc đã chạy vật lý đến `TASK SUCCESS`; log tham chiếu được
  lưu tại `docs/evidence/milestone7_runtime.txt`.
- [x] Toàn bộ 127 test đạt trong môi trường không còn process Gazebo/RViz cũ.
- [x] Có báo cáo tiếng Việt tại `docs/BAO_CAO_BAI_03.md`.
- [x] Có hướng dẫn chi tiết chụp sáu ảnh và quay video tại
  `docs/HUONG_DAN_DEMO_VA_CHUP_ANH.md`.
- [ ] Chèn sáu ảnh chụp thật vào báo cáo.
- [ ] Quay/upload video và điền link video.
- [ ] Tạo GitHub Public, điền link repository và commit sau khi người dùng
  duyệt toàn bộ nội dung.

Không tạo commit trong bước này theo yêu cầu. Ba mục chưa đánh dấu cần dữ liệu
do người dùng chụp/upload; source và quy trình tạo các dữ liệu đó đã sẵn sàng.

## 12. Thứ tự thực hiện bắt buộc

Không nên viết LLM prompt trước khi gripper và perception hoạt động độc lập.
Thứ tự an toàn là:

```text
M1 Scene
-> M2 Physical Gripper
-> M3 Camera
-> M4 Perception
-> M5 Perception-aware Skills
-> M6 State-aware Validator
-> M7 LLM Integration
-> M8 End-to-end Demo
```

Điểm kiểm soát quan trọng nhất là cuối M2 và M4. Nếu gripper chưa giữ được cube
vật lý hoặc camera chưa trả pose ổn định, chưa chuyển sang logic LLM.
