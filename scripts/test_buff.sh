#!/usr/bin/env bash
set -eo pipefail
task_repo=$(cd "$(dirname "$0")/.." && pwd)
if [[ -f /opt/ros/noetic/setup.bash ]]; then
  source /opt/ros/noetic/setup.bash
fi
set -u
task_build=${BUFF_TEST_BUILD_DIR:-"$task_repo/.cache/buff-tests"}
task_messages=""
if [[ "${BUFF_TEST_WITH_ROS:-ON}" == "ON" ]]; then
  task_message_ws="$task_repo/.cache/buff-ros-messages"
  mkdir -p "$task_message_ws/src"
  if [[ ! -e "$task_message_ws/src/rm_msgs" ]]; then
    ln -s "$task_repo/src/rm_msgs" "$task_message_ws/src/rm_msgs"
  fi
  if ! catkin_make -C "$task_message_ws" -j"${BUFF_TEST_JOBS:-2}" \
    -DCATKIN_ENABLE_TESTING=OFF -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    > "$task_message_ws/generate.log" 2>&1; then
    cat "$task_message_ws/generate.log"
    exit 1
  fi
  set +u
  source "$task_message_ws/devel/setup.bash"
  set -u
  task_messages="$task_message_ws/devel/include"
fi
cmake -S "$task_repo/scripts/buff-tests" -B "$task_build" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUFF_TEST_WITH_ROS="${BUFF_TEST_WITH_ROS:-ON}" \
  -DBUFF_MESSAGE_INCLUDE_DIR="$task_messages"
cmake --build "$task_build" --parallel "${BUFF_TEST_JOBS:-2}"
cd "$task_build"
ctest --output-on-failure
