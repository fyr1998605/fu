#!/bin/bash
echo "=== Binary check ==="
file "/home/fyr/ws_glut_vison2711（8.5）/devel/lib/rm_tracker/Fu" 2>&1
echo ""
echo "=== ldd openvino ==="
ldd "/home/fyr/ws_glut_vison2711（8.5）/devel/lib/rm_tracker/Fu" 2>&1 | grep -i openvino
echo ""
echo "=== OpenVINO 2022.3.0 headers ==="
ls /opt/intel/openvino_2022.3.0/runtime/include/openvino/openvino.hpp 2>&1
echo ""
echo "=== OpenVINO 2022.3.0 lib ==="
ls /opt/intel/openvino_2022.3.0/runtime/lib/intel64/libopenvino.so 2>&1
echo ""
echo "=== Model opset check ==="
grep -c 'version="opset11"' "/home/fyr/ws_glut_vison2711（8.5）/Model/fu_moxi/buff_repvgg.xml" 2>&1
echo "opset9 count:"
grep -c 'version="opset9"' "/home/fyr/ws_glut_vison2711（8.5）/Model/fu_moxi/buff_repvgg.xml" 2>&1
echo ""
echo "=== CMakeLists OpenVINO refs ==="
grep -c "openvino_2022" "/home/fyr/ws_glut_vison2711（8.5）/src/rm_tracker/CMakeLists.txt" 2>&1
echo "2024 refs:"
grep -c "openvino_2024" "/home/fyr/ws_glut_vison2711（8.5）/src/rm_tracker/CMakeLists.txt" 2>&1
echo ""
echo "=== DONE ==="