from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    world = LaunchConfiguration('world')

    #
    # Generate runtime URDF FIRST
    #
    generate_runtime_urdf = Node(
        package='fire_drone_bringup',
        executable='generate_runtime_urdf',
        output='screen'
    )

    #
    # Main bringup
    #
    new_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fire_drone_bringup'),
                'launch',
                'new.launch.py'
            ])
        ),
        launch_arguments={
            'world': world
        }.items()
    )

    #
    # Fire source simulator
    #
    fire_source = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'fire_drone_sim_cpp',
            'fire_source_sim',
            '--ros-args',
            '-p',
            ['world:=', world],
        ],
        output='screen'
    )

    #
    # PX4 takeoff command
    #
    takeoff_cmd = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            'cd ~/PX4-Autopilot && pxh -c "commander takeoff 3"'
        ],
        output='screen'
    )

    #
    # RTABMAP planner
    #
    rtabmap_planner = Node(
        package='fire_drone_controller',
        executable='fire_rtabmap_planner',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'world',
            default_value='forest'
        ),

        #
        # STEP 1:
        # Generate URDF BEFORE EVERYTHING
        #
        generate_runtime_urdf,

        #
        # STEP 2:
        # Start main launch after URDF generation
        #
        TimerAction(
            period=3.0,
            actions=[new_launch]
        ),

        #
        # STEP 3:
        # Start fire source
        #
        TimerAction(
            period=11.0,
            actions=[fire_source]
        ),

        #
        # STEP 4:
        # PX4 takeoff
        #
        TimerAction(
            period=15.0,
            actions=[takeoff_cmd]
        ),

        #
        # STEP 5:
        # Start planner
        #
        TimerAction(
            period=23.0,
            actions=[rtabmap_planner]
        ),
    ])
