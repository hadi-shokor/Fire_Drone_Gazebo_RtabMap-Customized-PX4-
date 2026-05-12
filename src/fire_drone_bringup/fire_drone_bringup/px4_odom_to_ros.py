#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
from scipy.spatial.transform import Rotation as R

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class PX4OdomToROS(Node):
    def __init__(self):
        super().__init__('px4_odom_to_ros')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_pub = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.callback,
            qos
        )

        # PX4 world frame: NED
        # ROS world frame: ENU
        self.world_transform = np.array([
            [0.0, 1.0, 0.0],   # x_enu = y_ned
            [1.0, 0.0, 0.0],   # y_enu = x_ned
            [0.0, 0.0, -1.0],  # z_enu = -z_ned
        ])

        # PX4 body frame: FRD
        # ROS body frame: FLU
        self.body_transform = np.array([
            [1.0, 0.0, 0.0],    # x_flu = x_frd
            [0.0, -1.0, 0.0],   # y_flu = -y_frd
            [0.0, 0.0, -1.0],   # z_flu = -z_frd
        ])

        self.get_logger().info(
            'PX4 odom converter started: VehicleOdometry NED/FRD -> ROS ENU/FLU, publishing /odom and odom->base_link'
        )

    def transform_orientation(self, px4_q):
        # PX4 quaternion order: [w, x, y, z]
        q_scipy = [
            float(px4_q[1]),
            float(px4_q[2]),
            float(px4_q[3]),
            float(px4_q[0]),
        ]

        # Rotation from PX4 NED world to PX4 FRD body
        r_ned_frd = R.from_quat(q_scipy)
        R_ned_frd = r_ned_frd.as_matrix()

        # Full conversion:
        # ROS ENU world + ROS FLU body
        R_enu_flu = self.world_transform @ R_ned_frd @ self.body_transform

        q_enu_flu = R.from_matrix(R_enu_flu).as_quat()  # [x, y, z, w]

        # Keep quaternion sign consistent
        if q_enu_flu[3] < 0.0:
            q_enu_flu = -q_enu_flu

        return q_enu_flu

    def callback(self, msg: VehicleOdometry):
        stamp = self.get_clock().now().to_msg()

        # Position: PX4 NED -> ROS ENU
        position_ned = np.array([
            float(msg.position[0]),
            float(msg.position[1]),
            float(msg.position[2]),
        ])
        position_enu = self.world_transform @ position_ned

        # Orientation: PX4 NED/FRD -> ROS ENU/FLU
        q_enu_flu = self.transform_orientation(msg.q)

        # Linear velocity
        velocity_raw = np.array([
            float(msg.velocity[0]),
            float(msg.velocity[1]),
            float(msg.velocity[2]),
        ])

        # velocity_frame:
        # 1 = NED
        # 3 = BODY_FRD
        if msg.velocity_frame == 1:
            velocity_enu = self.world_transform @ velocity_raw
        elif msg.velocity_frame == 3:
            velocity_enu = self.body_transform @ velocity_raw
        else:
            velocity_enu = self.world_transform @ velocity_raw

        # Angular velocity: PX4 FRD -> ROS FLU
        angular_velocity_frd = np.array([
            float(msg.angular_velocity[0]),
            float(msg.angular_velocity[1]),
            float(msg.angular_velocity[2]),
        ])
        angular_velocity_flu = self.body_transform @ angular_velocity_frd

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = float(position_enu[0])
        odom.pose.pose.position.y = float(position_enu[1])
        odom.pose.pose.position.z = float(position_enu[2])

        odom.pose.pose.orientation.x = float(q_enu_flu[0])
        odom.pose.pose.orientation.y = float(q_enu_flu[1])
        odom.pose.pose.orientation.z = float(q_enu_flu[2])
        odom.pose.pose.orientation.w = float(q_enu_flu[3])

        odom.twist.twist.linear.x = float(velocity_enu[0])
        odom.twist.twist.linear.y = float(velocity_enu[1])
        odom.twist.twist.linear.z = float(velocity_enu[2])

        odom.twist.twist.angular.x = float(angular_velocity_flu[0])
        odom.twist.twist.angular.y = float(angular_velocity_flu[1])
        odom.twist.twist.angular.z = float(angular_velocity_flu[2])

        # Simple covariance
        for i in range(6):
            odom.pose.covariance[i * 6 + i] = 0.01
            odom.twist.covariance[i * 6 + i] = 0.01

        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'

        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation = odom.pose.pose.orientation

        self.tf_pub.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = PX4OdomToROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
