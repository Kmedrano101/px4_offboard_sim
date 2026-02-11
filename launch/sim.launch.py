"""PX4 Offboard Simulation: Full simulation stack launcher.

Everything is driven from config/sim.yaml — the single source of truth.

Launches:
  1. MicroXRCE-DDS Agent (PX4 <-> ROS2 bridge)
  2. PX4 SITL with Gazebo
  3. ROS-Gazebo sensor bridge (generated from sim.yaml bridge section)
  4. Offboard control node (C++ state machine)
  5. Joy control node (keyboard + gamepad)
  6. Frame manager (Gazebo frame_id remapping)
  7. Static TF (map -> odom placeholder)

Usage:
  ros2 launch px4_offboard_sim sim.launch.py
  ros2 launch px4_offboard_sim sim.launch.py world:=baylands
  ros2 launch px4_offboard_sim sim.launch.py joy:=true
"""

import os
import shutil
import tempfile
import yaml
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ros_gz_bridge.actions import RosGzBridge

# ── Load sim config ──────────────────────────────────────────
_config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'sim.yaml')
with open(_config_path) as _f:
    CFG = yaml.safe_load(_f)


# ── Controller presets ────────────────────────────────────────
CONTROLLERS = {
    'tpro': {
        'config': 'joy_control_tpro.yaml',
        'joy_dev': '/dev/input/js1',
        'deadzone': 0.03,
    },
    'gamepad': {
        'config': 'joy_control.yaml',
        'joy_dev': '/dev/input/js0',
        'deadzone': 0.05,
    },
}


def _force_symlink(src, dst):
    """Create symlink dst -> src, removing whatever exists at dst."""
    if dst.is_symlink():
        dst.unlink()
    elif dst.is_dir():
        shutil.rmtree(dst)
    elif dst.exists():
        dst.unlink()
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.symlink_to(src)


def _resolve_model_instance(sim_model):
    """Derive Gazebo model instance name from PX4 sim model.

    PX4 convention: sim_model='gz_x500_depth' -> instance='x500_depth_0'
    """
    name = sim_model
    if name.startswith('gz_'):
        name = name[3:]
    return f'{name}_0'


def _generate_bridge_config(world, model_instance):
    """Generate ros_gz_bridge YAML config from sim.yaml bridge section.

    Resolves {world} and {model} placeholders in gz_topic paths.
    Writes to a temp file and returns its path.
    """
    bridge_cfg = CFG.get('bridge', {})
    entries = []

    for sensor in bridge_cfg.get('sensors', []):
        gz_topic = sensor['gz_topic'].format(world=world, model=model_instance)
        entries.append({
            'gz_topic_name': gz_topic,
            'ros_topic_name': sensor['ros_topic'],
            'ros_type_name': sensor['ros_type'],
            'gz_type_name': sensor['gz_type'],
            'direction': 'GZ_TO_ROS',
            'publisher_queue': sensor.get('queue', 10),
        })

    clock = bridge_cfg.get('clock')
    if clock:
        entries.append({
            'topic_name': '/clock',
            'ros_type_name': clock['ros_type'],
            'gz_type_name': clock['gz_type'],
            'direction': 'GZ_TO_ROS',
            'publisher_queue': clock.get('queue', 10),
        })

    # Write to a persistent temp file (cleaned up by OS)
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='gz_bridge_', delete=False
    )
    yaml.dump(entries, tmp, default_flow_style=False)
    tmp.close()
    return tmp.name


def _setup_simulation(context):
    """OpaqueFunction: create symlinks and generate bridge config at launch time."""
    px4_dir = Path(LaunchConfiguration('px4_autopilot_dir').perform(context))
    world = LaunchConfiguration('world').perform(context)
    pkg_share = Path(FindPackageShare('px4_offboard_sim').perform(context))

    gz_tree = px4_dir / 'Tools' / 'simulation' / 'gz'

    # ── Symlink worlds into PX4 ────────────────────────────
    worlds_dir = pkg_share / 'worlds'
    if worlds_dir.is_dir():
        for sdf in worlds_dir.glob('*.sdf'):
            _force_symlink(sdf, gz_tree / 'worlds' / sdf.name)

    # ── Symlink models into PX4 ────────────────────────────
    models_dir = pkg_share / 'models'
    if models_dir.is_dir():
        for model in models_dir.iterdir():
            if model.is_dir() and (model / 'model.sdf').exists():
                _force_symlink(model, gz_tree / 'models' / model.name)

    # ── Generate bridge config ─────────────────────────────
    model_instance = _resolve_model_instance(CFG['px4']['sim_model'])
    bridge_config_path = _generate_bridge_config(world, model_instance)

    # ── ros_gz_bridge node ─────────────────────────────────
    bridge = RosGzBridge(
        bridge_name='sim_gz_bridge',
        config_file=bridge_config_path,
        use_composition=False,
    )

    return [bridge]


def _launch_joy_nodes(context):
    """OpaqueFunction: launch joy_node + joy_control_node based on sim.yaml."""
    joy_cfg = CFG.get('joy', {})
    controller = joy_cfg.get('controller', 'tpro')
    joy_dev = joy_cfg.get('device', '')

    if controller not in CONTROLLERS:
        raise ValueError(
            f"Unknown controller '{controller}'. Choose from: {list(CONTROLLERS.keys())}")

    preset = CONTROLLERS[controller]

    if not joy_dev:
        joy_dev = preset['joy_dev']

    pkg_share = FindPackageShare('px4_offboard_sim').perform(context)
    config_file = os.path.join(pkg_share, 'config', preset['config'])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': preset['deadzone'],
            'autorepeat_rate': 20.0,
            'dev': joy_dev,
            'use_sim_time': True,
        }],
        output='screen',
    )

    joy_control_node = Node(
        package='px4_offboard_sim',
        executable='joy_control_node',
        name='joy_control',
        parameters=[config_file, {'use_sim_time': True}],
        output='screen',
    )

    return [joy_node, joy_control_node]


def generate_launch_description():
    pkg_share = FindPackageShare('px4_offboard_sim')

    # ── Resolve config values ────────────────────────────────
    px4_dir = os.path.expanduser(CFG['px4']['autopilot_dir'])
    dds_binary = os.path.expanduser(CFG['dds_agent']['binary'])

    # ── Arguments (all defaults from sim.yaml) ────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=CFG['world'],
        description='Gazebo world name (must match SDF filename without extension)',
    )

    px4_dir_arg = DeclareLaunchArgument(
        'px4_autopilot_dir',
        default_value=px4_dir,
        description='Absolute path to PX4-Autopilot checkout',
    )

    joy_arg = DeclareLaunchArgument(
        'joy',
        default_value='true',
        choices=['true', 'false'],
        description='Launch joy_node + joy_control_node for gamepad/keyboard input',
    )

    offboard_arg = DeclareLaunchArgument(
        'offboard',
        default_value='true',
        choices=['true', 'false'],
        description='Launch offboard control node',
    )

    frame_manager_arg = DeclareLaunchArgument(
        'frame_manager',
        default_value='true',
        choices=['true', 'false'],
        description='Launch frame manager for Gazebo frame_id remapping',
    )

    # ── PX4 paths ─────────────────────────────────────────────
    px4_build = os.path.join(px4_dir, 'build', 'px4_sitl_default')
    px4_binary = os.path.join(px4_build, 'bin', 'px4')
    px4_rootfs = os.path.join(px4_build, 'rootfs')
    px4_romfs = os.path.join(px4_dir, 'ROMFS', 'px4fmu_common')

    gz_models = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')
    gz_worlds = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'worlds')
    gz_plugins = os.path.join(px4_build, 'src', 'modules', 'simulation', 'gz_plugins')
    gz_server_config = os.path.join(
        px4_dir, 'src', 'modules', 'simulation', 'gz_bridge', 'server.config'
    )

    # ── Environment ───────────────────────────────────────────
    existing_gz_res = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    existing_gz_plug = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')

    install_prefix = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
    local_models = os.path.join(install_prefix, 'share', 'px4_offboard_sim', 'models')
    local_worlds = os.path.join(install_prefix, 'share', 'px4_offboard_sim', 'worlds')

    new_gz_resource = ':'.join(filter(None, [
        local_models, local_worlds, gz_models, gz_worlds, existing_gz_res,
    ]))
    new_gz_plugins = ':'.join(filter(None, [gz_plugins, existing_gz_plug]))

    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', new_gz_resource
    )
    set_gz_plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH', new_gz_plugins
    )
    set_gz_server_config = SetEnvironmentVariable(
        'GZ_SIM_SERVER_CONFIG_PATH', gz_server_config
    )
    set_rmw = SetEnvironmentVariable(
        'RMW_IMPLEMENTATION', CFG['rmw_implementation']
    )

    # ── MicroXRCE-DDS Agent ───────────────────────────────────
    dds_agent = ExecuteProcess(
        cmd=[
            dds_binary,
            CFG['dds_agent']['transport'],
            '-p', str(CFG['dds_agent']['port']),
        ],
        output='screen',
        name='micro_xrce_dds_agent',
    )

    # ── PX4 SITL ──────────────────────────────────────────────
    ds = CFG['drone_start']
    model_pose = f"{ds['x']},{ds['y']},{ds['z']},{ds['roll']},{ds['pitch']},{ds['yaw']}"

    px4_sitl = ExecuteProcess(
        cmd=[px4_binary, px4_romfs, '-d'],
        cwd=px4_rootfs,
        additional_env={
            'PX4_SIM_MODEL': CFG['px4']['sim_model'],
            'PX4_GZ_WORLD': LaunchConfiguration('world'),
            'PX4_GZ_MODELS': gz_models,
            'PX4_GZ_WORLDS': gz_worlds,
            'PX4_GZ_MODEL_POSE': model_pose,
            'PX4_GZ_NO_FOLLOW': '1' if CFG['px4']['gz_no_follow'] else '0',
        },
        output='screen',
        name='px4_sitl',
    )

    # ── Symlinks + Bridge (OpaqueFunction — runs at launch time) ──
    setup_sim = OpaqueFunction(function=_setup_simulation)

    # ── Static TF (map -> odom placeholder) ───────────────────
    map_frame = CFG['frames']['map_frame_id']
    body_frame = CFG['frames']['body_frame_id']
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', map_frame, '--child-frame-id', body_frame,
        ],
        parameters=[{'use_sim_time': True}],
        output='log',
    )

    # ── Offboard control node ─────────────────────────────────
    offboard_cfg = CFG.get('offboard', {})
    offboard_config_file = offboard_cfg.get('config', 'offboard_control.yaml')
    offboard_config = PathJoinSubstitution([pkg_share, 'config', offboard_config_file])
    offboard_node = Node(
        package='px4_offboard_sim',
        executable='offboard_control_node',
        name='offboard_control',
        parameters=[offboard_config, {'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('offboard')),
    )

    # ── Frame manager ─────────────────────────────────────────
    frame_manager_node = Node(
        package='px4_offboard_sim',
        executable='frame_manager.py',
        name='frame_manager',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('frame_manager')),
    )

    # ── Joy control (keyboard + gamepad) ──────────────────────
    joy_control_launch = OpaqueFunction(
        function=_launch_joy_nodes,
        condition=IfCondition(LaunchConfiguration('joy')),
    )

    return LaunchDescription([
        world_arg,
        px4_dir_arg,
        joy_arg,
        offboard_arg,
        frame_manager_arg,
        set_gz_resource_path,
        set_gz_plugin_path,
        set_gz_server_config,
        set_rmw,
        dds_agent,
        px4_sitl,
        setup_sim,
        static_tf,
        offboard_node,
        frame_manager_node,
        joy_control_launch,
    ])
