import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from hexapod_gazebo.urdf_utils import parse_urdf


"""
Send dummy joint states:

ros2 topic pub /hexapod/joint_values sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}},
  name: [
    'leg_1_coxa','leg_1_femur','leg_1_tibia',
    'leg_2_coxa','leg_2_femur','leg_2_tibia',
    'leg_3_coxa','leg_3_femur','leg_3_tibia',
    'leg_4_coxa','leg_4_femur','leg_4_tibia',
    'leg_5_coxa','leg_5_femur','leg_5_tibia',
    'leg_6_coxa','leg_6_femur','leg_6_tibia'
  ],
  position: [
    0.17, 0.35, -0.52,
    0.26, 0.44, -0.61,
    0.35, 0.52, -0.70,
    0.21, 0.38, -0.56,
    0.31, 0.49, -0.66,
    0.24, 0.42, -0.59
  ]
}"
"""


def generate_launch_description():

    pkg = get_package_share_directory("hexapod_gazebo")
    hardware_dir = os.path.join(pkg, "Hexapod-Hardware")
    urdf_path = os.path.join(hardware_dir, "hexapod.urdf")
    config_path = os.path.join(pkg, "config", "ros2_control.yaml")
    world_path = os.path.join(pkg, "worlds", "empty.sdf")

    robot_description = parse_urdf(urdf_path, hardware_dir, config_path)

    with open("/tmp/enriched_robot.urdf", "w") as f:
        f.write(robot_description)

    """
    # No need for use_sim_time=True in this setup: the timing is 
    # controller by the robot, Gazebo is just a visualizer with
    # physics.
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use Gazebo simulation clock",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    """

    device_id_arg = DeclareLaunchArgument(
        'joy_device_id',
        default_value='0',
        description='Joystick device ID (run joy_enumerate_devices to list available devices)',
    )

    joy_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('hexapod_gazebo'), 'launch', 'joy_teleop.launch.py'
            ])
        ]),
        launch_arguments={
            'joy_device_id': LaunchConfiguration('joy_device_id'),
        }.items(),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={
            "gz_args": ["-r ", world_path],
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            # "use_sim_time": use_sim_time,
            "use_sim_time": True,
        }],
        # Publishes to /controller_manager/robot_description so the
        # controller_manager receives the URDF over the topic it subscribes to.
        remappings=[("/robot_description", "/controller_manager/robot_description")],
    )

    """
    # Bridge /clock so nodes using use_sim_time receive Gazebo time.
    # In this architecture, the robot is the source of truth: listens
    # for commands, computes joint values and applies/streams them.
    # This node bridges /clock from Gazebo into ROS so that nodes 
    # using use_sim_time=True can receive simulation time, but no
    # node should use use_sim_time=True in this setup.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )
    """

    # Spawn the robot. Depends on robot_state_publisher being up, which starts immediately,
    # so the 3s delay is sufficient.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_hexapod",
        output="screen",
        arguments=[
            "-name",  "hexapod",
            "-topic", "/controller_manager/robot_description",
            "-z",     "0.20",
        ],
    )
    spawn_delayed = TimerAction(period=3.0, actions=[spawn_robot])

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    forward_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_command_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner])],
        )
    )

    spawn_fcc = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[forward_controller_spawner],
        )
    )

    relay_node = Node(
        package="hexapod_gazebo",
        executable="joint_command_bridge",
        name="joint_command_bridge",
        output="screen",
    )

    return LaunchDescription([
        # use_sim_time_arg,
        device_id_arg,
        gz_sim,
        robot_state_publisher,
        # clock_bridge,
        spawn_delayed,
        spawn_jsb,
        spawn_fcc,
        relay_node,
        joy_teleop
    ])