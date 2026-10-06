#!/usr/bin/env bash
# ROS 2 Humble setup files may read optional variables before defining them.
# Enable nounset only after both environments have been sourced.
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PLANNER="${PLANNER:-mock}"
COMMAND="${COMMAND:-Đưa khối đỏ sang vùng B.}"
GAZEBO_GUI="${GAZEBO_GUI:-true}"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-true}"

if [[ ! -f "${PROJECT_DIR}/install/setup.bash" ]]; then
  echo "Chưa có install/setup.bash. Hãy chạy ./scripts/build_humble.sh trước." >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/install/setup.bash"
set -u

if [[ "${PLANNER}" == "gemini" && -z "${GEMINI_API_KEY:-}" ]]; then
  echo "PLANNER=gemini yêu cầu biến môi trường GEMINI_API_KEY." >&2
  exit 2
fi

echo "Planner : ${PLANNER}"
echo "Command : ${COMMAND}"
echo "Scenario: scene_blocked.yaml"

exec ros2 launch ur3_robot_skills end_to_end.launch.py \
  "planner:=${PLANNER}" \
  "command:=${COMMAND}" \
  "gazebo_gui:=${GAZEBO_GUI}" \
  "launch_rviz:=${LAUNCH_RVIZ}"
