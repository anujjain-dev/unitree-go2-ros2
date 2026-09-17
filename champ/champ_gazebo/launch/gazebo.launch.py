import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution


def generate_launch_description():
    """Launch file for Ignition Fortress (Gazebo) simulation."""

    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    ros_control_file = LaunchConfiguration("ros_control_file")
    world_init_x = LaunchConfiguration("world_init_x")
    world_init_y = LaunchConfiguration("world_init_y")
    world_init_z = LaunchConfiguration("world_init_z")
    world_init_heading = LaunchConfiguration("world_init_heading")
    gazebo_world = LaunchConfiguration("world")
    skip_robot_state_publisher = LaunchConfiguration("skip_robot_state_publisher")
    
    gz_pkg_share = launch_ros.substitutions.FindPackageShare(package="champ_gazebo").find(
        "champ_gazebo"
    )

    declare_robot_name = DeclareLaunchArgument("robot_name", default_value="go2")
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="True")
    declare_headless = DeclareLaunchArgument("headless", default_value="False")
    declare_skip_robot_state_publisher = DeclareLaunchArgument(
        "skip_robot_state_publisher", default_value="False",
        description="Skip robot_state_publisher if already launched elsewhere"
    )
    declare_controller = DeclareLaunchArgument(
        "controller", default_value="champ",
        description="Locomotion controller: 'champ' (gait controller, effort trajectory "
                    "controller) or 'model' (RL policy, group effort controller). The two "
                    "ros2_control controllers claim the same command interfaces, so exactly "
                    "one is loaded.",
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=os.path.join(gz_pkg_share, "config/ros_control.yaml"),
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=os.path.join(gz_pkg_share, "worlds/default.sdf")
    )
    declare_world_name = DeclareLaunchArgument(
        "world_name", default_value="default",
        description="<world name=...> inside the world file; Ignition scopes sensor topics by it",
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.8")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )

    pkg_share = launch_ros.substitutions.FindPackageShare(package="champ_description").find("champ_description")
    default_model_path = os.path.join(pkg_share, "urdf/champ.urdf.xacro")

    declare_description_path = DeclareLaunchArgument(
        name="description_path", 
        default_value=default_model_path, 
        description="Absolute path to robot urdf file"
    )

    declare_description_args = DeclareLaunchArgument(
        name="description_args",
        default_value="",
        description="Extra xacro arguments for the description, e.g. 'laser:=true'",
    )

    config_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="champ_config"
    ).find("champ_config")
    
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    
    # Set ignition resource path for models
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(gz_pkg_share, 'worlds'), ':',
            os.path.join(gz_pkg_share, 'models'), ':',
            '/usr/share/ignition/ignition-gazebo6/worlds'
        ]
    )

    # Start Ignition Gazebo with the world file
    # Use 'ign gazebo' for Ignition Fortress, or 'gz sim' for newer versions
    start_ignition_cmd = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', gazebo_world,
             '--render-engine', 'ogre2'],
        output='screen',
        condition=IfCondition(PythonExpression(['not ', headless])),
    )
    
    start_ignition_headless_cmd = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-s', gazebo_world],
        output='screen',
        condition=IfCondition(headless),
    )

    # Robot description
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", LaunchConfiguration("description_path"), " ",
                     LaunchConfiguration("description_args")]),
            value_type=str,
        )
    }

    # Robot state publisher (skip if already launched by parent launch file)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        condition=IfCondition(PythonExpression(['not ', skip_robot_state_publisher])),
    )

    # Spawn robot in Ignition using ros_gz_sim
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-topic', '/robot_description',
            '-x', world_init_x,
            '-y', world_init_y,
            '-z', world_init_z,
            '-Y', world_init_heading,
        ],
        output='screen',
    )

    # Bridge for clock (Ignition -> ROS2)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
    )

    # Bridge for IMU data.
    # Like the contact sensors below, Ignition scopes sensor topics by world, model and
    # link and ignores any <topic> given on the sensor, so the full path has to be
    # rebuilt here. Bridging a bare '/imu/data' silently produces no messages at all.
    # sdformat lumps the fixed-jointed imu_link into base_link, so the sensor lives there.
    imu_ign_topic = [
        "/world/", LaunchConfiguration("world_name"),
        "/model/", robot_name,
        "/link/base_link/sensor/imu_sensor/imu",
    ]
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[imu_ign_topic + ["@sensor_msgs/msg/Imu[ignition.msgs.IMU"]],
        remappings=[(imu_ign_topic, "/imu/data")],
        output='screen',
    )

    # Bridge for ground-truth odometry (base twist in the body frame).
    # The RL policy needs base linear velocity, which the IMU does not provide.
    odom_gt_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom_ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
        output='screen',
    )

    # Bridge for laser scan
    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
        output='screen',
    )

    # Bridge for Velodyne point cloud (Ignition publishes to /velodyne_points/points)
    velodyne_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/velodyne_points/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'],
        output='screen',
        remappings=[('/velodyne_points/points', '/velodyne_points')],
    )

    # Foot contact sensors (Ignition -> ROS).
    # Ignition scopes contact sensor topics by world and model name and ignores
    # any <topic> given on the sensor, so the full path has to be rebuilt here.
    # champ's state_estimation node needs these to produce leg odometry; without
    # them /odom never leaves the origin and SLAM/Nav2 have nothing to work with.
    foot_contact_bridges = []
    for leg in ("lf", "rf", "lh", "rh"):
        ign_topic = [
            "/world/", LaunchConfiguration("world_name"),
            "/model/", robot_name,
            "/link/", leg, "_lower_leg_link",
            "/sensor/", leg, "_foot_contact/contact",
        ]
        foot_contact_bridges.append(Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f"{leg}_foot_contact_bridge",
            arguments=[ign_topic + ["@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts"]],
            remappings=[(ign_topic, f"/foot_contacts/{leg}")],
            output="screen",
        ))

    # Aggregates the four bridged contact topics into champ_msgs/ContactsStamped.
    foot_contacts_relay = Node(
        package="champ_gazebo",
        executable="foot_contacts_relay.py",
        name="foot_contacts_relay",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "contact_topics": ["/foot_contacts/lf", "/foot_contacts/rf",
                               "/foot_contacts/lh", "/foot_contacts/rh"],
        }],
    )

    # Load joint state broadcaster (with delay to wait for controller_manager)
    load_joint_state_controller = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                 'joint_states_controller'],
            output='screen',
        )],
    )

    # Load whichever joint controller the selected locomotion controller drives.
    # Both claim the same effort command interfaces, so only one may be active.
    is_champ = PythonExpression(["'", LaunchConfiguration("controller"), "' == 'champ'"])
    is_model = PythonExpression(["'", LaunchConfiguration("controller"), "' == 'model'"])

    load_joint_trajectory_effort_controller = TimerAction(
        period=6.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                 'joint_group_effort_controller'],
            output='screen',
            condition=IfCondition(is_champ),
        )],
    )

    # RL policy path: a plain group effort controller, since the policy runs its own PD
    # with the gains it was trained against.
    load_rl_effort_controller = TimerAction(
        period=6.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                 'rl_joint_effort_controller'],
            output='screen',
            condition=IfCondition(is_model),
        )],
    )

    return LaunchDescription(
        [
            declare_robot_name,
            declare_use_sim_time,
            declare_headless,
            declare_skip_robot_state_publisher,
            declare_controller,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_world_name,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            declare_description_path,
            declare_description_args,
            ign_resource_path,
            start_ignition_cmd,
            start_ignition_headless_cmd,
            robot_state_publisher,
            spawn_robot,
            clock_bridge,
            imu_bridge,
            odom_gt_bridge,
            scan_bridge,
            velodyne_bridge,
            *foot_contact_bridges,
            foot_contacts_relay,
            load_joint_state_controller,
            load_joint_trajectory_effort_controller,
            load_rl_effort_controller,
        ]
    )
