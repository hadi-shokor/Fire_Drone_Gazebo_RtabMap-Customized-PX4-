from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')

    imu_gz_topic = [
        '/world/', world,
        '/model/', model,
        '/link/base_link/sensor/imu_sensor/imu'
    ]

    runtime_urdf_path = '/tmp/fire_drone_runtime.urdf'

    with open(runtime_urdf_path, 'r') as infp:
        robot_description = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/velodyne_points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            imu_gz_topic + ['@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        ],
        remappings=[
            ('/velodyne_points/points', '/points_raw'),
            (imu_gz_topic, '/imu/data'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rtabmap_launch'),
                'launch',
                'rtabmap.launch.py'
            ])
        ),
        launch_arguments={
            'rtabmap_viz': 'true',
            'rviz': 'false',
            'subscribe_scan_cloud': 'true',
            'scan_cloud_topic': '/points_raw',

            'icp_odometry': 'false',
            'visual_odometry': 'false',
            'rgbd_sync': 'false',
            'depth': 'false',
            'stereo': 'false',

            'odom_topic': '/odom',
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',

            'approx_sync': 'true',
            'qos_scan': '2',
            'qos_odom': '2',
            'use_sim_time': 'true',
            'wait_for_transform': '5.0',

            'rtabmap_args':
                '--delete_db_on_start '
                '--ros-args '
                '-p Reg/Strategy:=2 '
                '-p RGBD/ProximityBySpace:=false '
                '-p RGBD/NeighborLinkRefining:=false '
                '-p RGBD/OptimizeFromGraphEnd:=true '
                '-p Kp/MaxFeatures:=-1 '
                '-p Vis/MaxFeatures:=0 '
                '-p Icp/PointToPlane:=true '
                '-p Icp/VoxelSize:=0.2 '
                '-p Icp/MaxCorrespondenceDistance:=1.0 '
                '-p Icp/Iterations:=20 '
                '-p publish_tf:=false '
                '-p Grid/CellSize:=0.10 '
                '-p Grid/RangeMax:=20.0 '
                '-p Grid/FromDepth:=false '
                '-p Grid/3D:=false '
                '-p Grid/RayTracing:=true '
        }.items()
    )

    px4_odom_to_ros = Node(
        package='fire_drone_bringup',
        executable='px4_odom_to_ros',
        name='px4_odom_to_ros',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    map_to_odom_static = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        '0', '0', '0',
        '0', '0', '0',
        'map',
        'odom'
    ],
    parameters=[{'use_sim_time': True}],
    output='screen'
)

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('fire_drone_bringup'),
                'rviz',
                'fire_drone_rtabmap.rviz'
            ])
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='baylands'),
        DeclareLaunchArgument('model', default_value='x500_fire_drone_0'),

        robot_state_publisher,
        bridge,
        rtabmap_launch,
        map_to_odom_static,
        px4_odom_to_ros,
        rviz,
    ])
