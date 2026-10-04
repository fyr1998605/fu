#!/usr/bin/env python3
import sys
import os

model_dir = "/home/fyr/ws_glut_vison2711（8.5）/Model/fu_moxi"

try:
    from openvino.runtime import Core
    print("OpenVINO Python API available")
    core = Core()
    print(f"OpenVINO version: {core.get_versions()}")
except ImportError as e:
    print(f"OpenVINO Python not available: {e}")
    sys.exit(1)

models_to_test = [
    "buff_repvgg.xml",
    "buff.onnx",
    "buff2026_3_3.onnx",
]

for name in models_to_test:
    path = os.path.join(model_dir, name)
    if not os.path.exists(path):
        print(f"\n{name}: FILE NOT FOUND")
        continue
    print(f"\n{'='*60}")
    print(f"Testing: {name}")
    try:
        model = core.read_model(path)
        print(f"  SUCCESS!")
        print(f"  Inputs ({len(model.inputs)}):")
        for inp in model.inputs:
            print(f"    name='{inp.get_any_name()}', shape={inp.get_partial_shape()}, dtype={inp.get_element_type()}")
        print(f"  Outputs ({len(model.outputs)}):")
        for out in model.outputs:
            print(f"    name='{out.get_any_name()}', shape={out.get_partial_shape()}, dtype={out.get_element_type()}")
    except Exception as e:
        print(f"  FAILED: {e}")