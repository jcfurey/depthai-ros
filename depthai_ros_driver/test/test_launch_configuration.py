from pathlib import Path
import runpy

import ament_index_python.packages
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_launch(monkeypatch, filename):
    monkeypatch.setattr(
        ament_index_python.packages,
        "get_package_share_directory",
        lambda package: str(PACKAGE_ROOT.parent / package),
    )
    return runpy.run_path(str(PACKAGE_ROOT / "launch" / filename))


@pytest.mark.parametrize(
    "overrides,expected_frame",
    [
        ({"tf_prefix": "front", "namespace": "robot"}, "oak"),
        ({"name": "rear_oak", "tf_prefix": "/robot/rear/"}, "rear_oak"),
        ({"rs_compat": "true", "tf_prefix": "front"}, "camera"),
        ({"tf_prefix": "front", "rviz_fixed_frame": "/odom/"}, "odom"),
    ],
)
def test_rviz_fixed_frame_resolves_to_camera_base_or_explicit_override(
    monkeypatch, overrides, expected_frame
):
    module = load_launch(monkeypatch, "driver.launch.py")
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for entity in module["generate_launch_description"]().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)

    rviz, urdf, _ = module["launch_setup"](context)
    display_arguments = [perform_substitutions(context, arg) for arg in rviz.cmd[1:5]]
    assert display_arguments[2:] == ["-f", expected_frame]
    if "rviz_fixed_frame" not in overrides:
        base_frame = dict(urdf.launch_arguments)["base_frame"]
        assert expected_frame == perform_substitutions(
            context, normalize_to_list_of_substitutions(base_frame)
        )


def test_rosbag_launch_fails_before_starting_processes(monkeypatch):
    module = load_launch(monkeypatch, "stereo_from_rosbag.launch.py")
    with pytest.raises(RuntimeError, match="not implemented"):
        module["launch_setup"](LaunchContext())

@pytest.mark.parametrize("filename", [
    "example_det2d_overlay.launch.py", "example_feature_3d.launch.py",
    "example_feature_tracker.launch.py", "example_seg_overlay.launch.py",
    "spatial_bb.launch.py", "thermal_temp.launch.py",
])
def test_filter_launch_uses_camera_namespace_and_forwards_selection(monkeypatch, filename):
    import launch_ros.actions
    from types import SimpleNamespace
    monkeypatch.setattr(ament_index_python.packages, "get_package_share_directory", lambda package: str(PACKAGE_ROOT.parent / package))
    monkeypatch.setattr(launch_ros.actions, "LoadComposableNodes", lambda **kwargs: SimpleNamespace(**kwargs))
    module = runpy.run_path(str(PACKAGE_ROOT.parent / "depthai_filters" / "launch" / filename))
    context = LaunchContext()
    context.launch_configurations.update({"name": "front", "namespace": "robot", "device_ip": "10.2.2.43", "autostart": "false", "tf_prefix": "robot_front"})
    for entity in module["generate_launch_description"]().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    driver, load = module["launch_setup"](context)
    arguments = dict(driver.launch_arguments)
    for key, value in {"device_ip": "10.2.2.43", "namespace": "robot", "autostart": "false", "tf_prefix": "robot_front"}.items():
        assert perform_substitutions(context, normalize_to_list_of_substitutions(arguments[key])) == value
    assert load.target_container == "/robot/front_container"
    for component in load.composable_node_descriptions:
        assert perform_substitutions(context, component.node_namespace) == "/robot/front"
        for source, target in component.remappings or []:
            if perform_substitutions(context, source) != "overlay":
                assert perform_substitutions(context, target).startswith("/robot/front/")


def test_filter_configuration_is_not_tied_to_oak_name():
    import yaml
    for path in (PACKAGE_ROOT.parent / "depthai_filters" / "config").glob("*.yaml"):
        assert "/oak" not in yaml.safe_load(path.read_text())
