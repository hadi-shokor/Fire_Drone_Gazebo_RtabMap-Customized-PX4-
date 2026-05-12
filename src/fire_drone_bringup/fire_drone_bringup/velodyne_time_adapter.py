import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class VelodyneTimeAdapter(Node):
    def __init__(self):
        super().__init__('velodyne_time_adapter')

        self.scan_period = 0.1  # 10 Hz

        self.sub = self.create_subscription(
            PointCloud2,
            '/points_raw',
            self.cloud_callback,
            10
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/points_raw_time',
            10
        )

        self.get_logger().info(
            'Velodyne time adapter started: /points_raw -> /points_raw_time'
        )

    def cloud_callback(self, msg):
        old_step = msg.point_step
        new_step = old_step + 4

        new_msg = PointCloud2()
        new_msg.header = msg.header
        new_msg.height = msg.height
        new_msg.width = msg.width
        new_msg.fields = list(msg.fields)
        new_msg.is_bigendian = msg.is_bigendian
        new_msg.is_dense = False
        new_msg.point_step = new_step
        new_msg.row_step = new_step * msg.width

        time_field = PointField()
        time_field.name = 'time'
        time_field.offset = old_step
        time_field.datatype = PointField.FLOAT32
        time_field.count = 1
        new_msg.fields.append(time_field)

        new_data = bytearray(new_msg.row_step * msg.height)

        for row in range(msg.height):
            for col in range(msg.width):
                old_index = row * msg.row_step + col * old_step
                new_index = row * new_msg.row_step + col * new_step

                point_bytes = msg.data[old_index:old_index + old_step]

                relative_time = 0.0
                time_bytes = struct.pack('<f', relative_time)

                new_data[new_index:new_index + old_step] = point_bytes
                new_data[new_index + old_step:new_index + new_step] = time_bytes

        new_msg.data = bytes(new_data)
        self.pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelodyneTimeAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
