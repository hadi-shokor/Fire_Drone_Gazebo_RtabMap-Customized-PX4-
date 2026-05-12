from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def px4_command_process(command, param1=0.0, param2=0.0, name="px4_command"):
    code = f"""
import rclpy, time
from px4_msgs.msg import VehicleCommand

rclpy.init()
node = rclpy.create_node('{name}')
pub = node.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

time.sleep(0.5)

for i in range(20):
    msg = VehicleCommand()
    msg.timestamp = int(node.get_clock().now().nanoseconds / 1000)
    msg.command = {command}
    msg.param1 = {param1}
    msg.param2 = {param2}
    msg.target_system = 1
    msg.target_component = 1
    msg.source_system = 1
    msg.source_component = 1
    msg.from_external = True
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.01)
    time.sleep(0.1)

node.destroy_node()
rclpy.shutdown()
"""

    return ExecuteProcess(
        cmd=['python3', '-c', code],
        output='screen'
    )


def generate_launch_description():

    arm_drone = px4_command_process(
        command=400,
        param1=1.0,
        name='arm_drone_command'
    )

    map_node = Node(
        package='fire_drone_navigation',
        executable='height_filtered_map_node',
        name='height_filtered_map_node',
        output='screen'
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fire_drone_navigation'),
                'launch',
                'fire_explore_nav.launch.py'
            ])
        )
    )

    set_offboard = px4_command_process(
        command=176,
        param1=1.0,
        param2=6.0,
        name='set_offboard_command'
    )

    return LaunchDescription([
        # Step 1: Arm first
        arm_drone,

        # Step 2: Start map after arming
        TimerAction(
            period=3.0,
            actions=[map_node]
        ),

        # Step 3: Start navigation after map
        TimerAction(
            period=8.0,
            actions=[nav_launch]
        ),

        # Step 4: Set OFFBOARD only after nav is already running
        TimerAction(
            period=18.0,
            actions=[set_offboard]
        ),
    ])
