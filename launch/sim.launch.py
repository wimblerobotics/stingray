# STINGRAY
import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

from launch.actions import (
    GroupAction,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    ld = LaunchDescription()

    package_path = get_package_share_directory("stingray")

    default_world = os.path.join(
        package_path,
        "worlds",
        "test_robot_world.sdf",
    )
    world = LaunchConfiguration("world")
    world_arg = DeclareLaunchArgument(
        "world", default_value=default_world, description="gz sim Test World"
    )
    ld.add_action(world_arg)

    xacro_file = os.path.join(package_path, "description/urdf", "robot.urdf.xacro")

    log_info_action = LogInfo(
        msg=[
            "world: [",
            world,
            "], package_path: [",
            package_path,
            "], xacro_file: [",
            xacro_file,
            "]",
        ]
    )
    ld.add_action(log_info_action)

    urdf_as_xml = xacro.process_file(
        xacro_file, mappings={"use_ros2_control": "true", "sim_mode": "true"}
    ).toxml()

    # Create a robot_state_publisher node
    params = {
        "ignore_timestamp": False,
        "robot_description": urdf_as_xml,
        #'publish_frequency' : 5.0,  - this has no effect. The topic is published on demand.
        "use_sim_time": True,
    }

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        namespace="/",
        parameters=[params],
    )
    ld.add_action(node_robot_state_publisher)

    # -- how to launch Gazebo UI:
    gazebo_ui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )
    ld.add_action(gazebo_ui)

    # spawn entity (robot model) in the Gazebo gz_sim
    # see arguments:  ros2 run ros_gz_sim create --helpshort
    spawn_sim_robot = Node(
        package="ros_gz_sim",
        executable="create",
        namespace="/",
        arguments=[
            "-name",
            "stingray",
            "-topic",
            "/robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.4",
            "-Y",
            "0.333",
            "-allow_renaming",
            "true",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    ld.add_action(spawn_sim_robot)

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="/",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
        # remappings don't work here. Use relay.
        # arguments=["diff_cont", "--controller-manager", "/controller_manager", "--ros-args", "--remap",  "/diff_cont/odom:=/odom"],
        # remappings=[('/diff_cont/odom','/odom')]
    )
    ld.add_action(diff_drive_spawner)

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="/",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
    )
    ld.add_action(joint_broad_spawner)

    bridge_params = os.path.join(package_path, "config", "gz_ros_bridge.yaml")
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
    )
    ld.add_action(ros_gz_bridge)

    # # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    # package_name = "stingray"  # <--- CHANGE ME from articubot_one

    # package_path = get_package_share_directory(package_name)

    # # You need to press "Startup" button in RViz when autostart=false
    # nav2 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(package_path, "launch", "navigation_launch.py")]
    #         # PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("nav2_bringup"),'launch','navigation_launch.py')]
    #     ),
    #     launch_arguments={"use_sim_time": "true", "autostart": "false"}.items(),
    # )

    # navsat_localizer = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(package_path, "launch", "dual_ekf_navsat.launch.py")]
    #     ),
    #     launch_arguments={"use_sim_time": "true"}.items(),
    # )

    # rsp = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(package_path, "launch", "rsp.launch.py")]
    #     ),
    #     launch_arguments={"use_sim_time": "true"}.items(),
    # )

    # joystick = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(package_path, "launch", "joystick.launch.py")]
    #     ),
    #     launch_arguments={"use_sim_time": "true"}.items(),
    # )

    # twist_mux = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(package_path, "launch", "twist_mux.launch.py")]
    #     ),
    #     launch_arguments={"use_sim_time": "true"}.items(),
    # )

    # slam_toolbox_params_file = os.path.join(
    #     package_path, "config", "mapper_params_online_async.yaml"
    # )

    # slam_toolbox = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [
    #             os.path.join(
    #                 get_package_share_directory("slam_toolbox"),
    #                 "launch",
    #                 "online_async_launch.py",
    #             )
    #         ]
    #     ),
    #     launch_arguments={
    #         "use_sim_time": "true",
    #         "slam_params_file": slam_toolbox_params_file,
    #     }.items(),
    # )

    # # Start Gazebo Harmonic (GZ, Ignition)
    # # -- set gazebo sim resource path for meshes and STLs:
    # gazebo_resource_path = SetEnvironmentVariable(
    #     name="GZ_SIM_RESOURCE_PATH",
    #     value=[
    #         os.path.join(package_path, "worlds"),
    #         ":" + os.path.join(package_path, "description"),
    #     ],
    # )

    # # -- where to find meshes:
    # gazebo_models_path = os.path.join(package_path, "description" "meshes")
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # # -- where to find the world SDF:
    # gazebo_arguments = LaunchDescription(
    #     [
    #         DeclareLaunchArgument(
    #             "world",
    #             default_value="test_robot_world",
    #             description="Gz sim Test World",
    #         ),
    #         # DeclareLaunchArgument('world', default_value='baylands',
    #         #                      description='Gz sim Baylands World'),
    #     ]
    # )

    # # No need to run controller_manager - it runs within Gazebo ROS2 Bridge.
    # # Only configure controllers, after the robot shows up live in GZ:

    # delayed_joint_broad_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=spawn_sim_robot,
    #         on_start=[joint_broad_spawner],
    #     )
    # )

    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=joint_broad_spawner,
    #         on_start=[diff_drive_spawner],
    #     )
    # )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace="/",
        # arguments=['-d', os.path.join(package_path, 'config', 'view_bot.rviz')],
        # arguments=['-d', os.path.join(package_path, 'config', 'map.rviz')],
        arguments=["-d", os.path.join(package_path, "config", "stingray.rviz")],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    ld.add_action(rviz)

    return ld
