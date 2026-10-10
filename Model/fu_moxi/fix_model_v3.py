#!/usr/bin/env python3
import sys
import os
import struct
import xml.etree.ElementTree as ET

MODEL_DIR = os.path.dirname(os.path.abspath(__file__))
XML_PATH = os.path.join(MODEL_DIR, "buff_repvgg.xml")
BIN_PATH = os.path.join(MODEL_DIR, "buff_repvgg.bin")
BAK_PATH = os.path.join(MODEL_DIR, "buff_repvgg.xml.bak")

print("=" * 60)
print("OpenVINO Model Fix Script")
print(f"Working directory: {MODEL_DIR}")
print("=" * 60)

# Step 1: Check OpenVINO Python API
print("\n[Step 1] Checking OpenVINO Python API...")
try:
    from openvino.runtime import Core
    core = Core()
    print(f"  OpenVINO is available!")
    print(f"  Version: {core.get_versions()}")
    ov_available = True
except ImportError as e:
    print(f"  OpenVINO Python not available: {e}")
    ov_available = False

# Step 2: Test loading backup XML (original, with opset11)
print("\n[Step 2] Testing original backup XML (opset11)...")
if ov_available:
    try:
        model = core.read_model(BAK_PATH)
        print(f"  SUCCESS! Inputs: {len(model.inputs)}, Outputs: {len(model.outputs)}")
    except Exception as e:
        print(f"  FAILED: {e}")
else:
    print("  Skipped (OpenVINO not available)")

# Step 3: Test loading ONNX files
print("\n[Step 3] Testing ONNX files...")
for onnx_name in ["buff.onnx", "buff2026_3_3.onnx"]:
    onnx_path = os.path.join(MODEL_DIR, onnx_name)
    if not os.path.exists(onnx_path):
        print(f"  {onnx_name}: NOT FOUND")
        continue
    print(f"  Testing {onnx_name}...")
    if ov_available:
        try:
            model = core.read_model(onnx_path)
            print(f"    SUCCESS! Inputs: {len(model.inputs)}, Outputs: {len(model.outputs)}")
            for inp in model.inputs:
                print(f"      Input: name='{inp.get_any_name()}', shape={inp.get_partial_shape()}, dtype={inp.get_element_type()}")
            for out in model.outputs:
                print(f"      Output: name='{out.get_any_name()}', shape={out.get_partial_shape()}, dtype={out.get_element_type()}")
        except Exception as e:
            print(f"    FAILED: {e}")
    else:
        print(f"    Skipped (OpenVINO not available)")

# Step 4: Analyze backup XML structure
print("\n[Step 4] Analyzing backup XML structure...")
tree = ET.parse(BAK_PATH)
root = tree.getroot()

for layer in root.findall(f".//layer[@type='Interpolate']"):
    layer_id = layer.get("id")
    version = layer.get("version")
    name = layer.get("name")
    print(f"\n  Interpolate layer id={layer_id}, name={name}, version={version}")
    for inp in layer.findall("input/port"):
        port_id = inp.get("id")
        precision = inp.get("precision")
        dims = [d.text for d in inp.findall("dim")]
        print(f"    Input port {port_id}: precision={precision}, dims={dims}")
    for out in layer.findall("output/port"):
        port_id = out.get("id")
        precision = out.get("precision")
        names = out.get("names", "")
        dims = [d.text for d in out.findall("dim")]
        print(f"    Output port {port_id}: precision={precision}, names='{names}', dims={dims}")

print("\n[Step 5] Finding Const layers...")
for layer in root.findall(f".//layer[@type='Const']"):
    layer_id = layer.get("id")
    data_elem = layer.find("data")
    if data_elem is not None:
        shape = data_elem.get("shape", "")
        elem_type = data_elem.get("element_type", "")
        offset = data_elem.get("offset", "")
        if "4" in shape:
            print(f"  Const id={layer_id}: element_type={elem_type}, shape={shape}, offset={offset}")
            for out in layer.findall("output/port"):
                port_id = out.get("id")
                precision = out.get("precision")
                names = out.get("names", "")
                print(f"    Output port {port_id}: precision={precision}, names='{names}'")

# Step 6: Create fixed XML
print("\n[Step 6] Creating fixed XML...")
tree = ET.parse(BAK_PATH)
root = tree.getroot()

changes_made = []

for layer in root.findall(f".//layer[@type='Interpolate']"):
    if layer.get("version") == "opset11":
        layer.set("version", "opset9")
        changes_made.append(f"Interpolate {layer.get('id')}: opset11 -> opset9")
        for inp in layer.findall("input/port"):
            if inp.get("id") == "1" and inp.get("precision") == "FP32":
                inp.set("precision", "I32")
                changes_made.append(f"  Input port 1: FP32 -> I32")

for layer in root.findall(f".//layer[@type='Const']"):
    data_elem = layer.find("data")
    if data_elem is not None:
        shape = data_elem.get("shape", "")
        elem_type = data_elem.get("element_type", "")
        if "4" in shape and elem_type == "f32":
            data_elem.set("element_type", "i32")
            offset = data_elem.get("offset", "")
            changes_made.append(f"Const {layer.get('id')}: element_type f32 -> i32 (offset={offset})")
            for out in layer.findall("output/port"):
                if out.get("precision") == "FP32":
                    out.set("precision", "I32")
                    changes_made.append(f"  Output port: FP32 -> I32")

tree.write(XML_PATH, xml_declaration=True, encoding="utf-8")
print(f"  Fixed XML written to: {XML_PATH}")

# Step 7: Fix binary file
print("\n[Step 7] Fixing binary file...")
with open(BIN_PATH, "rb") as f:
    bin_data = bytearray(f.read())

tree = ET.parse(BAK_PATH)
root = tree.getroot()
offsets_to_fix = []
for layer in root.findall(f".//layer[@type='Const']"):
    data_elem = layer.find("data")
    if data_elem is not None:
        shape = data_elem.get("shape", "")
        elem_type = data_elem.get("element_type", "")
        if "4" in shape and elem_type == "f32":
            offset = int(data_elem.get("offset", "0"))
            size = int(data_elem.get("size", "0"))
            n_elements = size // 4
            offsets_to_fix.append((offset, n_elements, layer.get("id")))

for offset, n_elements, layer_id in offsets_to_fix:
    print(f"  Converting Const {layer_id} at offset {offset}: {n_elements} float32 -> int32")
    floats = struct.unpack_from(f'{n_elements}f', bin_data, offset)
    ints = [int(round(v)) for v in floats]
    print(f"    Original floats: {floats}")
    print(f"    Converted ints: {ints}")
    struct.pack_into(f'{n_elements}i', bin_data, offset, *ints)

with open(BIN_PATH, "wb") as f:
    f.write(bin_data)
print(f"  Fixed binary written to: {BIN_PATH}")

# Step 8: Test fixed model
print("\n[Step 8] Testing fixed model...")
if ov_available:
    try:
        model = core.read_model(XML_PATH)
        print(f"  SUCCESS! Model loads! Inputs: {len(model.inputs)}, Outputs: {len(model.outputs)}")
        for inp in model.inputs:
            print(f"    Input: name='{inp.get_any_name()}', shape={inp.get_partial_shape()}, dtype={inp.get_element_type()}")
        for out in model.outputs:
            print(f"    Output: name='{out.get_any_name()}', shape={out.get_partial_shape()}, dtype={out.get_element_type()}")
        try:
            compiled = core.compile_model(model, "CPU")
            print(f"  Model compiles successfully!")
            req = compiled.create_infer_request()
            print(f"  Infer request created successfully!")
        except Exception as e:
            print(f"  Compilation FAILED: {e}")
    except Exception as e:
        print(f"  FAILED: {e}")
        import traceback
        traceback.print_exc()
else:
    print("  Skipped (OpenVINO not available)")

print("\n" + "=" * 60)
print("Changes made:")
for c in changes_made:
    print(f"  - {c}")
print("=" * 60)