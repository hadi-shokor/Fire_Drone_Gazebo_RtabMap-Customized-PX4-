from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')

    image_topic = [
        '/world/', world,
        '/model/', model,
        '/link/camera_link/sensor/camera/image'
    ]

    camera_info_topic = [
        '/world/', world,
        '/model/', model,
        '/link/camera_link/sensor/camera/camera_info'
    ]

    camera_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            image_topic + ['@sensor_msgs/msg/Image@gz.msgs.Image'],
            camera_info_topic + ['@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        ],
        output='screen'
    )

    fire_detector = Node(
        package='fire_drone_perception',
        executable='fire_detector',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    offboard_controller = Node(
        package='fire_drone_controller',
        executable='offboard_controller',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='baylands'),
        DeclareLaunchArgument('model', default_value='x500_fire_drone_0'),

        camera_bridge,
        fire_detector,
      #  offboard_controller,
    ])
