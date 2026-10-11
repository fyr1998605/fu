#!/usr/bin/env bash
set -eo pipefail
task_repo=$(cd "$(dirname "$0")/.." && pwd)
if [[ -f /opt/ros/noetic/setup.bash ]]; then
  source /opt/ros/noetic/setup.bash
fi
set -u
task_build=${BUFF_TEST_BUILD_DIR:-"$task_repo/.cache/buff-tests"}
cmake -S "$task_repo/scripts/buff-tests" -B "$task_build" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUFF_TEST_WITH_ROS="${BUFF_TEST_WITH_ROS:-ON}"
cmake --build "$task_build" --parallel "${BUFF_TEST_JOBS:-2}"
cd "$task_build"
ctest --output-on-failure
