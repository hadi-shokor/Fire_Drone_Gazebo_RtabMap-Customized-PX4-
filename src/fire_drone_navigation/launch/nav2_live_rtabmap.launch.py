from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    nav2_params = {
        'use_sim_time': True,

        'planner_plugins': ['GridBased'],
        'GridBased.plugin': 'nav2_navfn_planner::NavfnPlanner',

        'global_costmap.global_costmap.global_frame': 'map',
        'global_costmap.global_costmap.robot_base_frame': 'base_link',
    }

    return LaunchDescription([

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_params],
            remappings=[
                ('/map', '/fire/flyable_grid'),
                ('map', '/fire/flyable_grid'),
            ],
            output='screen'
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['planner_server']
            }],
            output='screen'
        ),
    ])
