from mujoco import mjcf

model = mjcf.from_path("Example_2_Existing_URDF_model/fiberthex/fiberthex.urdf")

mjcf_str = model.to_xml_string()

with open("convert_model.xml", "w") as f:
    f.write(mjcf_str)
    pass