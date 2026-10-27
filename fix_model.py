#!/usr/bin/env python3
import struct
import os

model_dir = "/home/fyr/ws_glut_vison2711（8.5）/Model/fu_moxi"
bin_path = os.path.join(model_dir, "buff_repvgg.bin")

with open(bin_path, 'rb') as f:
    data = f.read()

# Fix Const at offset 470496 (layer 46, feeds Interpolate 47)
offset = 470496
floats = struct.unpack_from('4f', data, offset)
print(f"Layer 46 Const (offset {offset}): floats={floats}")

ints = [int(round(v)) for v in floats]
print(f"  As integers: {ints}")

struct.pack_into('4i', data, offset, *ints)
print(f"  Patched to int32")

# Now check the other Const layers that might feed Interpolate 73
# Look for Const with shape="4" that have f32 type
# We need to find the Const that feeds Interpolate 73
# Let's search the XML for this

xml_path = os.path.join(model_dir, "buff_repvgg.xml")
with open(xml_path, 'r') as f:
    xml_content = f.read()

# Find all Const layers with element_type="f32" and shape="4"
# These are candidates for feeding Interpolate layers
import re

# Find Const layers with shape="4"
const_pattern = re.compile(
    r'<layer id="(\d+)" name="([^"]*)" type="Const"[^>]*>\s*'
    r'<data element_type="([^"]*)" shape="([^"]*)" offset="([^"]*)" size="([^"]*)"',
    re.DOTALL
)

for m in const_pattern.finditer(xml_content):
    layer_id = m.group(1)
    name = m.group(2)
    elem_type = m.group(3)
    shape = m.group(4)
    off = int(m.group(5))
    size = int(m.group(6))
    
    if '4' in shape.split(',') and elem_type == 'f32':
        vals = struct.unpack_from(f'{size//4}f', data, off)
        print(f"Const layer {layer_id} ({name}): type={elem_type}, shape={shape}, offset={off}, values={vals}")
        
        # Convert to int32
        int_vals = [int(round(v)) for v in vals]
        struct.pack_into(f'{size//4}i', data, off, *int_vals)
        print(f"  Patched to int32: {int_vals}")

with open(bin_path, 'wb') as f:
    f.write(data)

print("\nAll patches applied to binary file.")
print("Now patching XML...")

# Fix layer 46 XML: change element_type from f32 to i32 and precision from FP32 to I32
# Also fix the Interpolate layer 47 second input precision
xml_content = xml_content.replace(
    '<layer id="46" name="/model.6/Constant_output_0" type="Const" version="opset1">\n\t\t\t<data element_type="f32" shape="4" offset="470496" size="16" />\n\t\t\t<output>\n\t\t\t\t<port id="0" precision="FP32" names="/model.6/Constant_output_0">',
    '<layer id="46" name="/model.6/Constant_output_0" type="Const" version="opset1">\n\t\t\t<data element_type="i32" shape="4" offset="470496" size="16" />\n\t\t\t<output>\n\t\t\t\t<port id="0" precision="I32" names="/model.6/Constant_output_0">'
)

# Fix Interpolate 47 second input precision
xml_content = xml_content.replace(
    '<layer id="47" name="/model.6/Resize" type="Interpolate" version="opset9">',
    '<layer id="47" name="/model.6/Resize" type="Interpolate" version="opset9">'
)

# Fix port id="1" precision from FP32 to I32 in Interpolate 47
# Need to be careful - only fix the second input of the first Interpolate
old = '''<layer id="47" name="/model.6/Resize" type="Interpolate" version="opset9">
			<data mode="nearest" shape_calculation_mode="scales" coordinate_transformation_mode="asymmetric" nearest_mode="floor" antialias="false" pads_begin="0, 0, 0, 0" pads_end="0, 0, 0, 0" cube_coeff="-0.75" />
			<input>
				<port id="0" precision="FP32">
					<dim>1</dim>
					<dim>96</dim>
					<dim>20</dim>
					<dim>20</dim>
				</port>
				<port id="1" precision="FP32">
					<dim>4</dim>
				</port>
			</input>'''

new = '''<layer id="47" name="/model.6/Resize" type="Interpolate" version="opset9">
			<data mode="nearest" shape_calculation_mode="scales" coordinate_transformation_mode="asymmetric" nearest_mode="floor" antialias="false" pads_begin="0, 0, 0, 0" pads_end="0, 0, 0, 0" cube_coeff="-0.75" />
			<input>
				<port id="0" precision="FP32">
					<dim>1</dim>
					<dim>96</dim>
					<dim>20</dim>
					<dim>20</dim>
				</port>
				<port id="1" precision="I32">
					<dim>4</dim>
				</port>
			</input>'''

if old in xml_content:
    xml_content = xml_content.replace(old, new)
    print("Fixed Interpolate 47 second input precision to I32")
else:
    print("WARNING: Could not find exact match for Interpolate 47 input fix")

# Also check/fix Interpolate 73 second input
old2 = '''<layer id="73" name="/model.9/Resize" type="Interpolate" version="opset9">
			<data mode="nearest" shape_calculation_mode="scales" coordinate_transformation_mode="asymmetric" nearest_mode="floor" antialias="false" pads_begin="0, 0, 0, 0" pads_end="0, 0, 0, 0" cube_coeff="-0.75" />
			<input>
				<port id="0" precision="FP32">
					<dim>1</dim>
					<dim>48</dim>
					<dim>40</dim>
					<dim>40</dim>
				</port>
				<port id="1" precision="FP32">
					<dim>4</dim>
				</port>
			</input>'''

new2 = '''<layer id="73" name="/model.9/Resize" type="Interpolate" version="opset9">
			<data mode="nearest" shape_calculation_mode="scales" coordinate_transformation_mode="asymmetric" nearest_mode="floor" antialias="false" pads_begin="0, 0, 0, 0" pads_end="0, 0, 0, 0" cube_coeff="-0.75" />
			<input>
				<port id="0" precision="FP32">
					<dim>1</dim>
					<dim>48</dim>
					<dim>40</dim>
					<dim>40</dim>
				</port>
				<port id="1" precision="I32">
					<dim>4</dim>
				</port>
			</input>'''

if old2 in xml_content:
    xml_content = xml_content.replace(old2, new2)
    print("Fixed Interpolate 73 second input precision to I32")
else:
    print("WARNING: Could not find exact match for Interpolate 73 input fix")
    # Try to find what Const feeds Interpolate 73
    # Look for Const layers with shape="4" between layers 47 and 73
    print("Searching for Const that feeds Interpolate 73...")

with open(xml_path, 'w') as f:
    f.write(xml_content)

print("\nXML file updated.")
print("Done!")