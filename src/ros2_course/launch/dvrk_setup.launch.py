import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    # # Path to JSON file for dVRK main console
    # console_json_path = "~/dvrk2_ws/install/sawIntuitiveResearchKitAll/share/sawIntuitiveResearchKit/share/console/console-PSM1_KIN_SIMULATED.json"
    
    # # Path to RViz configuration file
    # rviz_config_path = "~/dvrk2_ws/install/dvrk_model/share/dvrk_model/rviz/PSM1.rviz"

    # Path to JSON file for dVRK main console
    console_json_path = os.path.expanduser("~/dvrk2_ws/install/sawIntuitiveResearchKitAll/share/sawIntuitiveResearchKit/share/console/console-PSM1_KIN_SIMULATED.json")
    
    # Path to RViz configuration file
    rviz_config_path = os.path.expanduser("~/dvrk2_ws/install/dvrk_model/share/dvrk_model/rviz/PSM1.rviz")


    return LaunchDescription([
                #ros2 run rviz2 rviz2 -d ~/dvrk2_ws/install/dvrk_model/share/dvrk_model/rviz/PSM1.rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            #arguments=[f'-d {rviz_config_path}']
            arguments=[f'-d {rviz_config_path}'],
            emulate_tty=True,
        ),

        #launch dummy marker
        # Node(
        #     package='ros2_course',
        #     executable='dummy_marker',
        #     output='screen',
        #     emulate_tty=True,
        # ),
        
        # Launch dVRK main console
        #ros2 run dvrk_robot dvrk_console_json -j ~/dvrk2_ws/install/sawIntuitiveResearchKitAll/share/sawIntuitiveResearchKit/share/console/console-PSM1_KIN_SIMULATED.json
        Node(
            package='dvrk_robot',
            executable='dvrk_console_json',
            name='dvrk_console',
            output='screen',
            arguments=['-j', f'{console_json_path}'],
            emulate_tty=True,
        ),


        # Launch ROS 2 joint and robot state publishers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('dvrk_model'), '/dvrk_state_publisher.launch.py'
            ]),
            launch_arguments={'arm': 'PSM1'}.items(),
        ),

        # Launch RViz with configuration
        #ros2 run rviz2 rviz2 -d ~/dvrk2_ws/install/dvrk_model/share/dvrk_model/rviz/PSM1.rviz
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     #arguments=[f'-d {rviz_config_path}']
        #     arguments=['-d ~/dvrk2_ws/install/dvrk_model/share/dvrk_model/rviz/PSM1.rviz']
        # ),
        
        # Launch Marker

    ])
#TODO figure out why rviz starts but does not show the robot model


