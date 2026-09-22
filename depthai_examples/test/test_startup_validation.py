"""Reject invalid example configuration before trying to open a device."""

import os
from pathlib import Path
import subprocess

import pytest


@pytest.mark.parametrize(
    "executable,parameters,reason",
    [
        ("rgb_compressed_publisher", ["profile:=999"], "valid encoder profile"),
        ("rgb_compressed_publisher", ["profile:=0"], "raw image output requires MJPEG"),
        ("rgbd_spatial_detections", ["monoWidth:=0"], "Invalid example parameters"),
        ("rgbd_spatial_detections", ["angularVelCovariance:=-1.0"], "Invalid example parameters"),
        ("rgbd_spatial_detections", ["enableDotProjector:=true"], "does not implement IR control"),
    ],
)
def test_invalid_startup_configuration(executable, parameters, reason, tmp_path):
    binary = Path(os.environ["DEPTHAI_EXAMPLE_BIN_DIR"]) / executable
    command = [str(binary), "--ros-args"]
    for value in parameters:
        command.extend(["-p", value])
    result = subprocess.run(
        command,
        env={**os.environ, "ROS_LOG_DIR": str(tmp_path)},
        capture_output=True,
        text=True,
        timeout=10,
        check=False,
    )
    assert result.returncode == 1, result.stdout + result.stderr
    assert reason in result.stderr
