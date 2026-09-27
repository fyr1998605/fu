#!/usr/bin/env python3
import sys
import os

model_dir = "/home/fyr/ws_glut_vison2711（8.5）/Model/fu_moxi"

print("Checking available model files...")
for f in sorted(os.listdir(model_dir)):
    if f.endswith(('.onnx', '.xml', '.bin')):
        fpath = os.path.join(model_dir, f)
        size = os.path.getsize(fpath)
        print(f"  {f}: {size/1024/1024:.2f} MB")

print("\n" + "="*60)

# Try OpenVINO
try:
    from openvino.runtime import Core
    print("OpenVINO Python bindings available")
    print(f"Version: {Core.__module__}")
    
    core = Core()
    
    # Test 1: Try loading the ONNX file directly
    for onnx_name in ['buff.onnx', 'buff2026_3_3.onnx']:
        onnx_path = os.path.join(model_dir, onnx_name)
        if os.path.exists(onnx_path):
            try:
                print(f"\nTrying to load {onnx_name}...")
                model = core.read_model(onnx_path)
                print(f"  SUCCESS! Inputs: {len(model.inputs)}, Outputs: {len(model.outputs)}")
                for inp in model.inputs:
                    print(f"    Input: {inp.get_any_name()}, shape={inp.get_partial_shape()}, dtype={inp.get_element_type()}")
                for out in model.outputs:
                    print(f"    Output: {out.get_any_name()}, shape={out.get_partial_shape()}, dtype={out.get_element_type()}")
            except Exception as e:
                print(f"  FAILED: {e}")
    
    # Test 2: Try loading the IR XML
    xml_path = os.path.join(model_dir, 'buff_repvgg.xml')
    bin_path = os.path.join(model_dir, 'buff_repvgg.bin')
    if os.path.exists(xml_path):
        try:
            print(f"\nTrying to load buff_repvgg.xml (IR)...")
            model = core.read_model(xml_path)
            print(f"  SUCCESS! Inputs: {len(model.inputs)}, Outputs: {len(model.outputs)}")
        except Exception as e:
            print(f"  FAILED: {e}")

except ImportError as e:
    print(f"OpenVINO Python not available: {e}")
    
    # Try onnxruntime
    try:
        import onnxruntime as ort
        print("\nTrying ONNX Runtime instead...")
        for onnx_name in ['buff.onnx', 'buff2026_3_3.onnx']:
            onnx_path = os.path.join(model_dir, onnx_name)
            if os.path.exists(onnx_path):
                try:
                    sess = ort.InferenceSession(onnx_path)
                    print(f"  {onnx_name}: OK, inputs={[i.name for i in sess.get_inputs()]}")
                except Exception as e:
                    print(f"  {onnx_name}: FAILED - {e}")
    except ImportError:
        print("Neither OpenVINO Python nor ONNX Runtime available")

print("\nDone.")