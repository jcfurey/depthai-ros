from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_single_camera_configs_apply_to_any_node_name():
    configs = sorted((PACKAGE_ROOT / "config").glob("*.yaml"))
    assert configs
    for config in configs:
        if config.name == "multicam_example.yaml":
            continue
        contents = yaml.safe_load(config.read_text(encoding="utf-8"))
        assert list(contents) == ["/**"], f"{config.name} is tied to a node name"


def test_default_and_low_bandwidth_profiles_are_explicit():
    default = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "driver.yaml").read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    assert default["driver"]["i_transport_profile"] == "AUTO"

    low_bandwidth = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "low_bandwidth.yaml").read_text(
            encoding="utf-8"
        )
    )["/**"]["ros__parameters"]
    assert low_bandwidth["driver"]["i_transport_profile"] == "LOW_BANDWIDTH"
    # Allow the driver to select a format supported by the connected platform.
    assert "i_low_bandwidth" not in low_bandwidth.get("stereo", {})


def test_oak_t_keeps_native_thermal_formats_raw():
    oak_t = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "oak_t.yaml").read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    assert oak_t["thermal"]["i_low_bandwidth"] is False


def test_launch_files_are_valid_python():
    for launch_file in sorted((PACKAGE_ROOT / "launch").glob("*.launch.py")):
        compile(
            launch_file.read_text(encoding="utf-8"),
            str(launch_file),
            "exec",
        )


def test_single_camera_wrappers_declare_and_forward_common_arguments():
    for launch_file in sorted((PACKAGE_ROOT / "launch").glob("*.launch.py")):
        source = launch_file.read_text(encoding="utf-8")
        if (
            '"driver.launch.py"' not in source
            or launch_file.name == "example_multicam.launch.py"
        ):
            continue

        setup_source, generate_source = source.split(
            "def generate_launch_description():", maxsplit=1
        )
        assert "**camera_launch_arguments()" in setup_source, launch_file.name
        assert "declare_camera_arguments()" not in setup_source, launch_file.name
        assert "declare_camera_arguments()" in generate_source, launch_file.name
