#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import numpy as np
from scipy.spatial.transform import Rotation as R

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class PX4OdomTFPublisher(Node):
    def __init__(self):
        super().__init__('px4_odom_to_ros')

        self.sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_frame = 'odom'
        self.base_frame = 'base_link'

        # PX4 world: NED → ROS world: ENU
        self.world_transform = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1],
        ])

        # PX4 body: FRD → ROS body: FLU
        self.body_transform = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1],
        ])

        self.get_logger().info(
            'PX4 VehicleOdometry converter started: NED/FRD -> ENU/FLU, publishing /odom and odom -> base_link'
        )

    def transform_orientation(self, px4_q):
        # PX4 q is [w, x, y, z]
        q_scipy = [
            px4_q[1],
            px4_q[2],
            px4_q[3],
            px4_q[0],
        ]

        r_ned_frd = R.from_quat(q_scipy)
        R_ned_frd = r_ned_frd.as_matrix()

        R_enu_flu = self.world_transform @ R_ned_frd @ self.body_transform

        q_enu = R.from_matrix(R_enu_flu).as_quat()  # [x, y, z, w]

        if q_enu[3] < 0:
            q_enu = -q_enu

        return q_enu

    def odom_callback(self, msg):
        now = self.get_clock().now().to_msg()

        position_ned = np.array([
            msg.position[0],
            msg.position[1],
            msg.position[2],
        ])

        position_enu = self.world_transform @ position_ned

        q_enu = self.transform_orientation(msg.q)

        velocity = np.array([
            msg.velocity[0],
            msg.velocity[1],
            msg.velocity[2],
        ])

        # PX4 velocity frame constants can differ by version.
        # For typical VehicleOdometry, velocity is usually in NED.
        velocity_enu = self.world_transform @ velocity

        angular_velocity_frd = np.array([
            msg.angular_velocity[0],
            msg.angular_velocity[1],
            msg.angular_velocity[2],
        ])

        angular_velocity_flu = self.body_transform @ angular_velocity_frd

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = float(position_enu[0])
        odom.pose.pose.position.y = float(position_enu[1])
        odom.pose.pose.position.z = float(position_enu[2])

        odom.pose.pose.orientation.x = float(q_enu[0])
        odom.pose.pose.orientation.y = float(q_enu[1])
        odom.pose.pose.orientation.z = float(q_enu[2])
        odom.pose.pose.orientation.w = float(q_enu[3])

        odom.twist.twist.linear.x = float(velocity_enu[0])
        odom.twist.twist.linear.y = float(velocity_enu[1])
        odom.twist.twist.linear.z = float(velocity_enu[2])

        odom.twist.twist.angular.x = float(angular_velocity_flu[0])
        odom.twist.twist.angular.y = float(angular_velocity_flu[1])
        odom.twist.twist.angular.z = float(angular_velocity_flu[2])

        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame

        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = PX4OdomTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
