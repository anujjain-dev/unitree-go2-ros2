import os

import yaml

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    description_path = LaunchConfiguration("description_path")
    base_frame = "base_link"

    config_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="go2_config"
    ).find("go2_config")
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="go2_description"
    ).find("go2_description")
    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    ros_control_config = os.path.join(
        config_pkg_share, "/config/ros_control/ros_control.yaml"
    )
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    rl_config = os.path.join(config_pkg_share, "config/rl/policy.yaml")

    # The checkpoint lives in the RL config; read it here so it can also be overridden
    # from the command line without duplicating the default in two places.
    with open(rl_config) as f:
        default_checkpoint = yaml.safe_load(f)["rl_policy_node"]["ros__parameters"]["checkpoint"]
    default_model_path = os.path.join(descr_pkg_share, "xacro/robot.xacro")
    # Use SDF world file for Ignition Fortress
    default_world_path = os.path.join(config_pkg_share, "worlds/default.sdf")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2_robot", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=ros_control_config,
        description="Ros control config path",
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path,
        description="Absolute path to the Ignition world (.sdf) to load",
    )
    # Ignition scopes sensor topics by the <world name=...> inside the world file, so the
    # IMU and foot-contact bridges have to be told what it is. Loading a world whose name
    # is not "default" without setting this leaves those topics unbridged and silent.
    declare_world_name = DeclareLaunchArgument(
        "world_name", default_value="default",
        description="The <world name=...> declared inside the world file",
    )

    declare_controller = DeclareLaunchArgument(
        "controller",
        default_value="champ",
        description="Locomotion controller: 'champ' for the CHAMP gait controller, "
                    "'model' for the IsaacLab-trained RL policy",
    )
    declare_checkpoint = DeclareLaunchArgument(
        "checkpoint",
        default_value=default_checkpoint,
        description="rsl_rl checkpoint to run when controller:=model",
    )

    declare_headless = DeclareLaunchArgument(
        "headless", default_value="False",
        description="Run Ignition without its GUI. Worth using for SLAM/Nav2 runs: the "
                    "GUI costs more CPU than the physics does.",
    )

    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )

    declare_laser = DeclareLaunchArgument(
        "laser", default_value="false", description="Mount the Hokuyo 2D scanner (publishes /scan)"
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.275")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )

    # Set Gazebo system plugin path for ros2_control
    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
    )

    
    description_args = ["laser:=", LaunchConfiguration("laser")]

    bringup_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_bringup"),
                "launch",
                "bringup.launch.py",
            )
        ),
        launch_arguments={
            "description_path": default_model_path,
            "description_args": description_args,
            "joints_map_path": joints_config,
            "links_map_path": links_config,
            "gait_config_path": gait_config,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "gazebo": "true",
            "lite": LaunchConfiguration("lite"),
            "rviz": LaunchConfiguration("rviz"),
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "false",
            "close_loop_odom": "true",
            "controller": LaunchConfiguration("controller"),
        }.items(),
    )

    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_gazebo"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "world": LaunchConfiguration("world"),
            "world_name": LaunchConfiguration("world_name"),
            "world_init_x": LaunchConfiguration("world_init_x"),
            "world_init_y": LaunchConfiguration("world_init_y"),
            "world_init_z": LaunchConfiguration("world_init_z"),
            "world_init_heading": LaunchConfiguration("world_init_heading"),
            "headless": LaunchConfiguration("headless"),
            "description_path": default_model_path,
            "description_args": description_args,
            "skip_robot_state_publisher": "True",
            "controller": LaunchConfiguration("controller"),
        }.items(),
    )

    # Replaces the CHAMP gait controller when controller:=model. champ_bringup keeps the
    # state estimator and EKFs running either way, so /odom, TF, SLAM and Nav2 are
    # unaffected by the switch.
    rl_policy_node = Node(
        package="go2_config",
        executable="rl_policy_node.py",
        name="rl_policy_node",
        output="screen",
        parameters=[
            rl_config,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"checkpoint": LaunchConfiguration("checkpoint")},
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("controller"), "' == 'model'"])
        ),
    )

    return LaunchDescription(
        [
            gz_plugin_path,
            declare_use_sim_time,
            declare_rviz,
            declare_robot_name,
            declare_lite,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_world_name,
            declare_controller,
            declare_checkpoint,
            declare_headless,
            declare_gui,
            declare_laser,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            bringup_ld,
            gazebo_ld,
            rl_policy_node,

        ]
    )
