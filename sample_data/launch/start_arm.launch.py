from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

params_file = os.path.join(
    get_package_share_directory('sample_data'), 'config', 'arms.yaml')

def generate_launch_description():
    # Define the nodes to be launched
    sample_node = Node(
        package='sample_data',
        executable='sample',
        name='sample_node',
        output='screen'
    )

    master_node = Node(
        package='arx_r5_controller',
        executable='R5Controller',
        name='arm_master',
        output='screen',
        parameters=[params_file],
    )

    follow_node = Node( 
        package='arx_r5_controller',
        executable='R5Controller',
        name='arm_follow',
        output='screen',
        parameters=[params_file],
    )

    # camera_node_top = Node(
    #     package='realsense2_camera',
    #     namespace='camera1',
    #     executable='realsense2_camera_node',
    #     name='camera',
    #     parameters=[{'serial_no': '230322271473',  # Replace with your camera's serial number
    #                 'device_type': 'd405',  # Replace with your camera model
    #                 }],
    #     output='screen'
    # )

    # camera_node_gripper = Node(
    #     package='realsense2_camera',
    #     namespace='camera2',
    #     executable='realsense2_camera_node',
    #     name='camera',
    #     parameters=[{'serial_no': '230322277180',  # Replace with your camera's serial number
    #                 'device_type': 'd405',          # Replace with your camera model
    #                 }],
    #     output='screen'
    # )

    # node2 = Node(
    #     package='your_package_name',
    #     executable='your_node_executable2',
    #     name='your_node_name2',
    #     output='screen'
    # )

    # Add the nodes to the LaunchDescription
    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file),
        sample_node,
        master_node,
        follow_node

    ])