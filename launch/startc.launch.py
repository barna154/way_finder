import os
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='way_finder',
            executable='megoldas2',
            output='screen',
            parameters=[{
                'Csapatnev': 'Racsecar',
                'Azonosito': '1',
                'debug': False,
            }]
        )
    ])

    start_rviz_2d_overlay = False

    try:
        package_path = get_package_prefix('rviz_2d_overlay_plugins')

        str_overlay = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
            FindPackageShare("way_finder"), '/string_rviz_overlay.launch.py'])
        )
        start_rviz_2d_overlay = True
    except:
        print("rviz_2d_overlay_plugins nem található")
        start_rviz_2d_overlay = False


    if start_rviz_2d_overlay:
        return LaunchDescription([
            megoldas2,
            str_overlay
        ])
    else:
        return LaunchDescription([
            megoldas2,
            LogInfo(msg="Hiba: rviz_2d_overlay_plugins nem található"),
            LogInfo(msg="Install: sudo apt install ros-humble-rviz-2d-overlay*"),
        ])