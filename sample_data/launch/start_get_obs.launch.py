from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import os

params_file = os.path.join(
    get_package_share_directory('sample_data'), 'config', 'arms.yaml')

def generate_launch_description():
    # Define the nodes to be launched

    follow_node = Node( 
        package='arx_r5_controller',
        executable='R5Controller',
        name='arm_follow',
        output='screen',
        parameters=[params_file],
    )

    camera_node_top = Node(
        package='realsense2_camera',
        namespace='camera1',
        executable='realsense2_camera_node',
        name='camera',
        parameters=[{'serial_no': '230322271473',  # Replace with your camera's serial number
                    'device_type': 'd405',  # Replace with your camera model
                    'enable_sync'   : True,
                    'depth_module.color_profile': '640x480x15',
                    'enable_depth': False,          # Disable depth stream
                    'enable_infra': False,          # Disable infrared stream
                    'enable_infra1': False,          # Disable infrared stream
                    'enable_infra2': False,          # Disable infrared stream
                    }],
        output='screen'
    )

    camera_node_gripper = Node(
        package='realsense2_camera',
        namespace='camera2',
        executable='realsense2_camera_node',
        name='camera',
        parameters=[{'serial_no': '230322277180',  # Replace with your camera's serial number
                    'device_type': 'd405',          # Replace with your camera model
                    'enable_sync'   : True,
                    'depth_module.color_profile': '640x480x15',
                    'enable_depth': False,          # Disable depth stream
                    'enable_infra': False,          # Disable infrared stream
                    'enable_infra1': False,          # Disable infrared stream
                    'enable_infra2': False,          # Disable infrared stream
                    
                    }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file),
        camera_node_top,
        camera_node_gripper,
        follow_node,
    ])