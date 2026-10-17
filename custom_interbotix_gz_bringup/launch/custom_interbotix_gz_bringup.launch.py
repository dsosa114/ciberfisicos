from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # General arguments
    tf_prefix = LaunchConfiguration("tf_prefix")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_use_params = LaunchConfiguration("rviz_use_params")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    initial_positions_file = LaunchConfiguration("initial_positions_file")
    use_world_frame = LaunchConfiguration("use_world_frame")
    fixed_frame_name = LaunchConfiguration("fixed_frame_name")
    use_gripper = LaunchConfiguration("use_gripper")


    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "rviz_config.rviz"]
    )

    robot_initial_positions_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", initial_positions_file]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "sim_gz:=true",
            " ",
            "initial_positions_file:=",
            robot_initial_positions_file,
            " ",
            "use_gripper:=",
            use_gripper,
            " ",
            "use_world_frame:=",
            use_world_frame,
            " ",
            "fixed_frame_name:=",
            fixed_frame_name,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": use_sim_time}, robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(
            PythonExpression(["'", launch_rviz, "' == 'true' and '", rviz_use_params, "' == 'false'"])
        ),
    )

    rviz_node_with_params = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(
            PythonExpression(["'", launch_rviz, "' == 'true' and '", rviz_use_params, "' == 'true'"])
        ),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node, rviz_node_with_params],
        ),
        condition=IfCondition(launch_rviz),
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
    )

    # Spawn controllers
    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "10",
            ]
            + inactive_flags
            + controllers,
            condition=IfCondition(start_joint_controller),
        )
    
    controllers_active = [
        "gripper_controller",
    ]
    controllers_inactive = []

    controller_spawners = [controller_spawner(controllers_active)]



    # Gazebo nodes
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch", "/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": 'empty.sdf -r -v 4',
        }.items(),
    )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="create_ur",
        arguments=["-name", "px150", "-topic", "robot_description"],
        output="screen",
    )

    # Make the /clock topic available in ROS
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    nodes_to_start = [
        robot_state_publisher_node,
        gazebo,
        gazebo_spawn_robot,
        gz_sim_bridge,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_stopped,
        initial_joint_controller_spawner_started,
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="custom_interbotix_xarms_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="px150.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gripper",
            default_value="true",
            description="Enable gripper in the robot description and start its controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_use_params",
            default_value="true",
            description="Start RViz with use_sim_time and robot_description parameters?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time", 
            default_value="true", 
            description="Start nodes with use_sim_time parameter set to true"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_file", 
            default_value="initial_positions.yaml", 
            description="Initial joint positions file name, located in the config folder of the description package."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_world_frame", 
            default_value="true", 
            description="Use the world frame as the fixed frame in the robot description."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fixed_frame_name", 
            default_value="world", 
            description="Name of the fixed frame to use in the robot description."
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])