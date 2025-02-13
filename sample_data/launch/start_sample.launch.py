from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the nodes to be launched
    sample_node = Node(
        package='sample_data',
        executable='sample',
        name='sample_node',
        output='screen'
    )

    # master_node = Node(
    #     package='sample_data',
    #     executable='master',
    #     name='master_node',
    #     output='screen'
    # )

    # follow_node = Node( 
    #     package='sample_data',
    #     executable='follow',
    #     name='follow_node',
    #     output='screen'
    # )

    camera_node_top = Node(
        package='realsense2_camera',
        namespace='camera1',
        executable='realsense2_camera_node',
        name='camera',
        parameters=[{'serial_no': '230322271473',  # Replace with your camera's serial number
                    'device_type': 'd405',  # Replace with your camera model
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
                    }],
        output='screen'
    )

    # node2 = Node(
    #     package='your_package_name',
    #     executable='your_node_executable2',
    #     name='your_node_name2',
    #     output='screen'
    # )

    # Add the nodes to the LaunchDescription
    return LaunchDescription([
        sample_node,
        # camera_node_gripper,
        camera_node_top,
        camera_node_gripper,
    ])