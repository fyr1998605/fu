#!/usr/bin/env bash
set -euo pipefail
task_repo=$(cd "$(dirname "$0")/.." && pwd)
task_build=${BUFF_TEST_BUILD_DIR:-"$task_repo/.cache/buff-tests"}
mkdir -p "$task_build"
"${CXX:-g++}" -std=c++17 -O2 -I "$task_repo/src/rm_tracker/include" \
  -I /usr/include/eigen3 \
  "$task_repo/src/rm_tracker/test/buff_prediction_comparison.cpp" \
  "$task_repo/src/rm_tracker/src/buff_big_variable_speed_predictor.cpp" \
  "$task_repo/src/rm_tracker/src/buff_big_ransac_predictor.cpp" \
  "$task_repo/src/rm_tracker/src/buff_zju_ransac.cpp" \
  -o "$task_build/buff_prediction_comparison"
"$task_build/buff_prediction_comparison"
