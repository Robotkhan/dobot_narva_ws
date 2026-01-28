import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Robot info
    robot_name = 'me6_robot'
    package_name = 'my_me6_pkg'
    urdf_name = 'me6_robot.urdf'
    initial_joint_controller_arg = DeclareLaunchArgument(
        "initial_joint_controller",
        default_value = "joint_trajectory_controller",
        description="Robot controller to start.",
    )
    runtime_config_package_arg = DeclareLaunchArgument(
        "runtime_config_package",
        default_value="my_me6_pkg",
        description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
    )
    controllers_file_arg = DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_me6_pkg"), 
                    "config", 
                    "me6_controllers.yaml"
                ]
            ),
            description="YAML file with the controllers configuration.",
    )
    start_joint_controller_arg = DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    description_package_arg = DeclareLaunchArgument(
            "description_package",
            default_value="my_me6_pkg",
            description="Description package with robot URDF/XACRO files. Usually the argument \
            is not set, it enables use of a custom description.",
        )
    
    initial_positions_file_arg = DeclareLaunchArgument(
            "initial_positions_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_me6_pkg"),
                    "config",
                    "initial_positions.yaml",
                ]
            ),
            description="YAML file (absolute path) with the robot's initial joint positions.",
    )
    
    description_file_arg = DeclareLaunchArgument(
            "description_file",
            default_value="me6_robot.urdf",
            description="URDF/XACRO description file with the robot.",
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Gazebo world file' 
    )
    
    # Paths
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    description_package = LaunchConfiguration("description_package")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    description_file = LaunchConfiguration("description_file")
    controllers_file = LaunchConfiguration("controllers_file")
    world = LaunchConfiguration('world')
    rviz_config_path = os.path.join(
        get_package_share_directory("me6_moveit_config"),
        "config",
        "moveit.rviz",
    )

    # Moveit config 
    moveit_config = (
        MoveItConfigsBuilder("custom_robot", package_name="me6_moveit_config")
        .robot_description(file_path="config/me6_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/me6_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description= True, publish_robot_description_semantic=True, publish_planning_scene=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    initial_positions_file_abs = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", initial_positions_file]
    )
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
            " ",
            "initial_positions_file:=",
            initial_positions_file_abs,
        ]
    )
    # robot_description = {"robot_description": robot_description_content}

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str
        )
    }
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world]
        }.items(),
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            moveit_config.robot_description, {'use_sim_time': True}
        ],
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', robot_name,
            '-string', robot_description_content,
            '-z', '0.1',
            '-static', 'true'
        ],
        parameters=[{'use_sim_time': True}]
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock", 
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",  
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
        ],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': True}]
    )

    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
        parameters=[{'use_sim_time': True}]
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
        parameters=[{'use_sim_time': True}]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )
    use_sim_time={"use_sim_time": True}
    config_dict = moveit_config.to_dict()
    config_dict.update(use_sim_time)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )
    return LaunchDescription([
        initial_joint_controller_arg,
        runtime_config_package_arg,
        controllers_file_arg,
        start_joint_controller_arg,
        description_package_arg,
        initial_positions_file_arg,
        description_file_arg,
        world_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        gz_sim_bridge,
        joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_started,
        initial_joint_controller_spawner_stopped,
        rviz_node,
        move_group_node,

    ])
