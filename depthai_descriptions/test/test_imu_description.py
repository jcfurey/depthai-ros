from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro


@pytest.mark.parametrize(
    "model",
    ["OAK-D", "OAK-D-POE", "OAK-D-PRO", "OAK-D-LITE", "OAK-D-S2", "OAK-D-PRO-W"],
)
def test_imu_description_keeps_model_mesh_and_connected_imu_frame(model):
    source = Path(__file__).resolve().parents[1] / "urdf" / "base_descr.urdf.xacro"
    doc = xacro.process_file(
        str(source),
        mappings={
            "camera_name": "front",
            "base_frame": "oak",
            "camera_model": model,
            "has_imu": "true",
        },
    )
    root = ET.fromstring(doc.toxml())
    assert root.find(".//mesh").attrib["filename"].endswith(f"/{model}.stl")
    joint = root.find("joint[@name='front_imu_joint']")
    assert joint.find("parent").attrib["link"] == "oak"
    assert joint.find("child").attrib["link"] == "front_imu_frame"
    if model in {"OAK-D-LITE", "OAK-D-S2", "OAK-D-PRO-W"}:
        # These models previously used the PRO alias for IMU extrinsics.
        assert joint.find("origin").attrib["xyz"] == "-0.008 -0.037945 -0.00079"
