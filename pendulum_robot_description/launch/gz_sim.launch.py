from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    robot_name = "pendulum_robot"
    robot_description_pkg = get_package_share_path(f"{robot_name}_description")

    gz_launch_path = os.path.join(get_package_share_path("ros_gz_sim"),
                                  'launch')

    gz_bridge_config_path = os.path.join(robot_description_pkg,
                                         'config', 'gz_bridge.yaml')
    
    gz_world_path = "empty.sdf"
    
    urdf_path = os.path.join(robot_description_pkg,
                             'urdf', f'{robot_name}.urdf.xacro')

    rviz_config_path = os.path.join(robot_description_pkg,
                                    'rviz', 'config.rviz')
    
    default_robot_config_file = os.path.join(robot_description_pkg,
                                             'config', 'parameters.yaml')

    robot_parameters_file_arg = DeclareLaunchArgument(
        'robot_parameters_file',
        default_value=default_robot_config_file,
        description="Full path to robot parameters file."

    )

    robot_parameters_file = LaunchConfiguration('robot_parameters_file')

    robot_description = ParameterValue(Command(
        ['xacro ',
          urdf_path,
          ' ',
          'robot_parameters_file:=',
          robot_parameters_file
          ]), value_type= str)

    robot_state_publisher_node = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        name='joint_state_publisher_gui',
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz_node = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path]    
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gz_launch_path,
            "/gz_sim.launch.py"
        ]), launch_arguments={'gz_args': f'{gz_world_path} -r -v 4 --render-engine ogre2'}.items()
    )

    gz_entity_create_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description']
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file':gz_bridge_config_path}]
    )

    return LaunchDescription([
        robot_parameters_file_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        gz_sim_launch,
        gz_entity_create_node,
        gz_bridge_node
    ])