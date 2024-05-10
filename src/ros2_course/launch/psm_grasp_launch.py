from launch import LaunchDescription
from launch_ros.actions import Node
# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         get_package_share_directory('dvrk_model'), '/dvrk_state_publisher.launch.py'
        #     ]),
        #     launch_arguments={'arm': 'PSM1'}.items(),
        # ),

        
        # Node(
        #     package='ros2_course',
        #     executable='dummy_marker',
        #     output='screen',
        #     emulate_tty=True,
        # ),

        #the above two nodes are temporarily moved to the dvrk_setup.launch.py file
        Node(
            package='ros2_course',
            executable='psm_grasp',
            output='screen',
            emulate_tty=True,
        ),
    ])