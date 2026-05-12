from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fire_drone_navigation'),
                'launch',
                'nav2_live_rtabmap.launch.py'
            ])
        )
    )

    frontier_planner = Node(
        package='fire_drone_navigation',
        executable='fire_frontier_planner',
        output='screen'
    )

    nav2_planner = Node(
        package='fire_drone_navigation',
        executable='fire_nav2_planner',
        output='screen'
    )

    path_follower = Node(
        package='fire_drone_controller',
        executable='nav2_path_follower',
        output='screen'
    )

    return LaunchDescription([
        frontier_planner,

        TimerAction(
            period=5.0,
            actions=[nav2]
        ),

        TimerAction(
            period=10.0,
            actions=[path_follower]
        ),

        TimerAction(
            period=15.0,
            actions=[nav2_planner]
        ),
    ])
