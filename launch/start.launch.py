from launch import LaunchDescription
from launch.actions import LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix


def generate_launch_description():

    megoldas2 = Node(
        package='way_finder',
        executable='megoldas2',
        name='megoldas',
        output='screen'
    )

    actions = [megoldas2]

    try:
        get_package_prefix('rviz_2d_overlay_plugins')

        overlay_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('way_finder'),
                '/launch/string_rviz_overlay.launch.py'
            ])
        )

        actions.append(overlay_launch)

    except Exception:
        actions.append(
            LogInfo(msg="Hiba: rviz_2d_overlay_plugins nem található")
        )
        actions.append(
            LogInfo(msg="Install: sudo apt install ros-humble-rviz-2d-overlay*")
        )

    return LaunchDescription(actions)