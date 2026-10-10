#!/usr/bin/env python3
"""
Fix OpenVINO IR model: convert Interpolate layers from opset11 to opset9,
and change Const data types from f32 to i32 for sizes inputs.
"""
import struct
import re
import os

MODEL_DIR = "/home/fyr/ws_glut_vison2711（8.5）/Model/fu_moxi"
XML_BAK = os.path.join(MODEL_DIR, "buff_repvgg.xml.bak")
XML_OUT = os.path.join(MODEL_DIR, "buff_repvgg.xml")
BIN_PATH = os.path.join(MODEL_DIR, "buff_repvgg.bin")

print("=" * 60)
print("OpenVINO IR Model Fix Script")
print("=" * 60)

# 1. Read backup XML
with open(XML_BAK, 'r') as f:
    xml_content = f.read()

print(f"\n1. Read backup XML: {len(xml_content)} bytes")

# 2. Read binary data
with open(BIN_PATH, 'rb') as f:
    bin_data = bytearray(f.read())

print(f"2. Read binary: {len(bin_data)} bytes")

# 3. Find and fix all Interpolate layers
# Change version="opset11" to version="opset9" for Interpolate layers
interpolate_count = 0

def fix_interpolate_version(match):
    global interpolate_count
    interpolate_count += 1
    full_match = match.group(0)
    fixed = full_match.replace('version="opset11"', 'version="opset9"')
    layer_id = re.search(r'id="(\d+)"', full_match)
    if layer_id:
        print(f"   Interpolate layer id={layer_id.group(1)}: opset11 -> opset9")
    return fixed

# Match Interpolate layers with version="opset11"
xml_content = re.sub(
    r'<layer[^>]*type="Interpolate"[^>]*version="opset11"[^>]*>',
    fix_interpolate_version,
    xml_content
)

# Also handle self-closing tags
xml_content = re.sub(
    r'<layer[^>]*type="Interpolate"[^>]*version="opset11"[^>]*/>',
    lambda m: m.group(0).replace('version="opset11"', 'version="opset9"'),
    xml_content
)

print(f"3. Fixed {interpolate_count} Interpolate layers: opset11 -> opset9")

# 4. Find all Const layers that feed Interpolate with f32 data
# These are Const layers with element_type="f32" that have shape matching size inputs
# We need to find the ones with shape="4" (or other sizes used by Interpolate)

# Strategy: find all Const layers with element_type="f32" and shape that matches
# an Interpolate's sizes input. The key ones have shape="4".

# First, let's find all Interpolate layers and their input structures
print("\n4. Analyzing Interpolate layers and their Const inputs...")

# Find all Const layers with f32 and collect their info
# Pattern: <layer id="N" name="..." type="Const" version="opset1">
#           <data element_type="f32" shape="SHAPE" offset="OFFSET" size="SIZE" />
const_pattern = re.compile(
    r'<layer\s+id="(\d+)"\s+name="([^"]*)"\s+type="Const"[^>]*>\s*'
    r'<data\s+element_type="f32"\s+shape="([^"]*)"\s+offset="(\d+)"\s+size="(\d+)"\s*/>\s*'
    r'<output>\s*<port[^>]*precision="FP32"[^>]*>',
    re.DOTALL
)

f32_consts = []
for match in const_pattern.finditer(xml_content):
    layer_id = int(match.group(1))
    name = match.group(2)
    shape = match.group(3)
    offset = int(match.group(4))
    size = int(match.group(5))
    f32_consts.append({
        'id': layer_id,
        'name': name,
        'shape': shape,
        'offset': offset,
        'size': size,
        'start': match.start(),
        'end': match.end()
    })
    print(f"   Const id={layer_id}, name='{name}', shape='{shape}', offset={offset}, size={size}")

# 5. Identify which f32 Const layers feed Interpolate layers
# Look for Const layers with shape="4" (or multi-element shapes that match Interpolate inputs)
# These are the ones we need to convert

# Also find Interpolate layers to understand their expected input types
interpolate_pattern = re.compile(
    r'<layer\s+id="(\d+)"\s+name="([^"]*)"\s+type="Interpolate"[^>]*>.*?</layer>',
    re.DOTALL
)

# Find Const layers that are specifically for Interpolate sizes
# These typically have shape="4" (for 4-dimensional input/output)
# or shape="2" (for 2-dimensional)

# Let's find ALL Const layers that feed into Interpolate
# by looking at the model structure

# Actually, the key insight is: the Const layers that need fixing are those
# that are consumed by Interpolate layers as their "sizes" or "scales" input.
# These Const layers typically have shapes like "4" (for 4D tensors) and contain
# values like [1.0, 2.0, 2.0, 1.0] or similar scaling factors.

# Let's find Const layers with small shapes that could be Interpolate sizes
target_consts = []
for const in f32_consts:
    shape_str = const['shape']
    # Check if this looks like a sizes input for Interpolate
    # Shapes like "4", "2" (single dimension) could be sizes
    # Also check for shapes like "1,2,2,1" (multi-dimensional sizes)
    if ',' not in shape_str:
        # Single dimension shape - could be Interpolate sizes
        dims = int(shape_str.strip())
        if dims <= 10:  # Small enough to be a sizes tensor
            target_consts.append(const)
            print(f"   -> TARGET: Const id={const['id']} shape='{shape_str}' offset={const['offset']}")

print(f"\n5. Found {len(target_consts)} Const layers to convert from f32 to i32")

# 6. Now convert the Const layers
# For each target Const:
# a. Convert binary data from float32 to int32
# b. Update XML: element_type f32 -> i32, precision FP32 -> I32

for const in target_consts:
    offset = const['offset']
    size_bytes = const['size']
    n_elements = size_bytes // 4  # f32 = 4 bytes, i32 = 4 bytes
    
    print(f"\n6. Processing Const id={const['id']} at offset={offset}, size={size_bytes}:")
    
    # Read current float values from binary
    if offset + size_bytes <= len(bin_data):
        floats = struct.unpack_from(f'{n_elements}f', bin_data, offset)
        print(f"   Current float values: {floats}")
        
        # Convert to int32 (round to nearest integer)
        ints = [int(round(v)) for v in floats]
        print(f"   Converted to int32:   {ints}")
        
        # Write back as int32
        struct.pack_into(f'{n_elements}i', bin_data, offset, *ints)
        print(f"   Binary data converted successfully")
        
        # Verify
        verify_ints = struct.unpack_from(f'{n_elements}i', bin_data, offset)
        print(f"   Verified int32 values: {verify_ints}")
    else:
        print(f"   WARNING: offset+size exceeds binary file size!")
        continue
    
    # Update XML for this Const layer
    # Change element_type="f32" -> element_type="i32"
    # Change precision="FP32" -> precision="I32"
    
    # Find and replace in the XML
    const_pattern_xml = re.compile(
        rf'(<layer\s+id="{const["id"]}"\s+name="{re.escape(const["name"])}"\s+type="Const"[^>]*>\s*'
        r'<data\s+)element_type="f32"(\s+shape="{re.escape(const["shape"])}"\s+offset="{const["offset"]}"\s+size="{const["size"]}"\s*/>\s*'
        r'<output>\s*<port[^>]*precision=")FP32(")',
        re.DOTALL
    )
    
    new_xml = const_pattern_xml.sub(
        rf'\1element_type="i32"\2\3I32\4',
        xml_content
    )
    
    if new_xml == xml_content:
        print(f"   WARNING: Could not find Const layer {const['id']} in XML for replacement!")
    else:
        print(f"   XML updated for Const layer {const['id']}")
        xml_content = new_xml

# 7. Fix Interpolate layers - change sizes input precision from FP32 to I32
# For each Interpolate layer, find the second input port (the sizes input) and change its precision

print("\n7. Fixing Interpolate input precisions...")

# Find all Interpolate layers in the modified XML
interpolate_pattern = re.compile(
    r'(<layer\s+id="(\d+)"\s+name="([^"]*)"\s+type="Interpolate"[^>]*>)\s*'
    r'(.*?)\s*</layer>',
    re.DOTALL
)

for match in interpolate_pattern.finditer(xml_content):
    full_start = match.start()
    layer_header = match.group(1)
    layer_id = match.group(2)
    layer_name = match.group(3)
    layer_body = match.group(4)
    
    # Check if this Interpolate has a second input with precision="FP32"
    # that needs to be changed to I32
    # Pattern: second input port with precision="FP32" and small shape like "4"
    
    # Find the <input>...</input> block
    input_match = re.search(r'<input>(.*?)</input>', layer_body, re.DOTALL)
    if input_match:
        input_block = input_match.group(1)
        
        # Find the second port (port id="1") 
        # Pattern: <port id="1" precision="FP32"> with shape
        port1_match = re.search(
            r'(<port\s+id="1"\s+)precision="FP32"(\s*>)',
            input_block
        )
        
        if port1_match:
            # Check if this port has a shape that matches sizes (small number like 4)
            port_content = input_block[port1_match.start():port1_match.end()+50]
            shape_match = re.search(r'<dim>(\d+)</dim>', port_content)
            if shape_match:
                dim_size = int(shape_match.group(1))
                if dim_size <= 10:  # It's a sizes input
                    print(f"   Interpolate id={layer_id}: changing port 1 precision FP32 -> I32 (shape dim={dim_size})")
                    
                    # Replace in the full XML
                    old = port1_match.group(0)
                    new = old.replace('precision="FP32"', 'precision="I32"')
                    
                    # Need to find this specific occurrence in the full XML
                    # Use a more targeted approach
                    search_start = match.start()
                    search_end = match.end()
                    
                    # Find the second input port within this Interpolate layer
                    layer_section = xml_content[search_start:search_end]
                    
                    # Replace the second port's precision
                    old_port = '<port id="1" precision="FP32">'
                    new_port = '<port id="1" precision="I32">'
                    
                    count = layer_section.count(old_port)
                    if count > 0:
                        layer_section = layer_section.replace(old_port, new_port)
                        xml_content = xml_content[:search_start] + layer_section + xml_content[search_end:]
                        print(f"      Replaced {count} occurrence(s)")
                    else:
                        print(f"      No matching port found in layer")
                else:
                    print(f"   Interpolate id={layer_id}: port 1 has shape dim={dim_size} (not a sizes input)")

# 8. Write the modified XML
print("\n8. Writing modified XML...")
with open(XML_OUT, 'w') as f:
    f.write(xml_content)
print(f"   Written to: {XML_OUT}")

# 9. Write the modified binary
print("\n9. Writing modified binary...")
with open(BIN_PATH, 'wb') as f:
    f.write(bin_data)
print(f"   Written to: {BIN_PATH}")

# 10. Verification
print("\n10. Verification...")

# Check opset versions
opset11_count = xml_content.count('version="opset11"')
opset9_count = xml_content.count('version="opset9"')
print(f"    opset11 count: {opset11_count}")
print(f"    opset9 count: {opset9_count}")

# Check element_type
f32_count = len(re.findall(r'element_type="f32"[^>]*shape="4"', xml_content))
i32_count = len(re.findall(r'element_type="i32"[^>]*shape="4"', xml_content))
print(f"    f32 Const with shape=4: {f32_count}")
print(f"    i32 Const with shape=4: {i32_count}")

# Check Interpolate precisions
interpolate_ports = re.findall(
    r'type="Interpolate".*?<port id="1" precision="(\w+)"',
    xml_content, re.DOTALL
)
print(f"    Interpolate port 1 precisions: {interpolate_ports}")

print("\n" + "=" * 60)
print("Fix complete! Try running: roslaunch rm_node start_fan_bag.launch")
print("=" * 60)