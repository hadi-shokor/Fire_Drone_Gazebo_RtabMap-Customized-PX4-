#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from px4_msgs.msg import OffboardControlMode, VehicleCommand


class InitialScanMission(Node):
    def __init__(self):
        super().__init__('initial_scan_mission')

        self.target_altitude = self.declare_parameter('target_altitude', 3.0).value

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        self.counter = 0
        self.takeoff_sent = False

        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info('Commander takeoff + offboard heartbeat node started.')

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # No position/yaw setpoints from this node
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_pub.publish(msg)

    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0,
    ):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)

        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.command_pub.publish(msg)

    def loop(self):
        self.counter += 1

        # Keep offboard heartbeat alive
        self.publish_offboard_mode()

        if self.counter == 10:
            self.get_logger().info('Arming...')
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0
            )

        elif self.counter == 20:
            self.get_logger().info('Switching to OFFBOARD...')
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0
            )

        elif self.counter == 30 and not self.takeoff_sent:
            self.get_logger().info(f'Commander takeoff to {self.target_altitude:.1f} m...')
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                0.0,
                0.0,
                0.0,
                float('nan'),
                float('nan'),
                float('nan'),
                self.target_altitude
            )
            self.takeoff_sent = True


def main(args=None):
    rclpy.init(args=args)
    node = InitialScanMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
