import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    apriltag_node = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource([os.path.join(
        	get_package_share_directory('isaac_ros_apriltag'), 'launch'),
        	'/isaac_ros_apriltag_usb_cam.launch.py']),
    	launch_arguments={'target_frame': 'carrot1'}.items(),
    )
    
    vision_node = Node(
        package='vision2024',
        name='sim',
        namespace='',
        executable='listener',
    )

    
    return LaunchDescription([
        apriltag_node,
		vision_node,
    ])