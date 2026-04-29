from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_path


def prepare_nodes(context):

    model_name = LaunchConfiguration('model_name').perform(context)
    model_dict = {'subject_with_mesh': 'humanSubjectWithMesh.urdf'}

    package_name = f'human_models'
    model_description_pkg = get_package_share_path(package_name)

    urdf_path = os.path.join(model_description_pkg,
                             'urdf', f'{model_dict[model_name]}')
    
    rviz_config_path = os.path.join(model_description_pkg,
                                    'rviz', 'config.rviz')

    
    model_description = ParameterValue(Command(
        ['xacro', 
         ' ',
         urdf_path,
        ]), value_type=str)

    model_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description':model_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    return model_state_publisher_node, joint_state_publisher_gui_node, rviz2_node

def generate_launch_description():

    # Declare launch arguments for Xacro parameters
    model_name_arg = DeclareLaunchArgument(
        'model_name', 
        default_value= 'subject_with_mesh',
        description='Robot to display: joints, vehicle'
    )

    return LaunchDescription([
        model_name_arg,
        OpaqueFunction(function=prepare_nodes)
    ])
