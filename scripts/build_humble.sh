#!/usr/bin/env bash
set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOTIQ_DESCRIPTION_DIR="${PROJECT_DIR}/third_party/robotiq_ros/grippers/robotiq_description"

source /opt/ros/humble/setup.bash

if [[ ! -d "${ROBOTIQ_DESCRIPTION_DIR}" ]]; then
  echo "Thiếu dependency Robotiq. Hãy chạy: vcs import third_party < dependencies.repos" >&2
  exit 2
fi

# gz_ros2_control 0.7.x expects mimic/multiplier joint parameters inside the
# ros2_control block. Apply the tracked compatibility patch exactly once.
if ! grep -q "Bai03 mimic compatibility v2" \
  "${ROBOTIQ_DESCRIPTION_DIR}/urdf/2f_common.ros2_control.xacro"; then
  patch -d "${ROBOTIQ_DESCRIPTION_DIR}" -p1 \
    < "${PROJECT_DIR}/patches/robotiq_gazebo_mimic_state.patch"
fi

# ROS 2 Humble on Ubuntu 22.04 is built against the system CMake 3.22.
# A pip-installed CMake 3.27+ removes legacy FindPythonInterp used by
# python_cmake_module/launch_testing and can break otherwise valid packages.
export PATH="/usr/bin:${PATH}"
hash -r

cmake --version | head -n 1

colcon build --symlink-install \
  --base-paths \
    src \
    third_party/robotiq_ros/grippers/robotiq_description \
  --packages-select \
    robotiq_description \
    ur_simulation_gz \
    ur3_workcell_description \
    ur3_llm_control \
    ur3_perception \
    ur3_robot_skills \
  --cmake-args -DBUILD_TESTING=OFF
