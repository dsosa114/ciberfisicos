from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    battery_node = Node(
        package="final_project_helpers",
        executable="battery_model",
        name="battery_node"
    )

    table_1_node = Node(
        package="final_project_helpers",
        executable='table',
        name='mesa_1_node',
        parameters=[{'table_id': 'mesa_1'}]
    )

    table_2_node = Node(
        package="final_project_helpers",
        executable='table',
        name='mesa_2_node',
        parameters=[{'table_id': 'mesa_2'}]
    )

    conveyor_node = Node(
        package="final_project_helpers",
        executable='conveyor',
        name='conveyor_node'
    )
    return LaunchDescription([
        battery_node,
        table_1_node,
        table_2_node,
        conveyor_node
    ])