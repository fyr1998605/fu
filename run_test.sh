#!/bin/bash
cd "/home/fyr/ws_glut_vison2711（8.5）"
python3 test_ov.py > /tmp/test_ov_output.txt 2>&1
echo "EXIT=$?" >> /tmp/test_ov_output.txt