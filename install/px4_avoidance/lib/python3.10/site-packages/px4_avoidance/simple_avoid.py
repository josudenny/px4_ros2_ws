#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import ObstacleDistance, OffboardControlMode, TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
import math

class SimpleAvoidance(Node):

    def __init__(self):
        super().__init__('simple_avoidance')

        # Subscribers
        self.create_subscription(
            ObstacleDistance,
            '/fmu/out/obstacle_distance',
            self.obstacle_callback,
            10
        )

        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.front_clear = True
        self.counter = 0

        self.get_logger().info("Simple Obstacle Avoidance Node Started")

    def obstacle_callback(self, msg):
        # Check FRONT sector (around 0 degrees)
        mid = len(msg.distances) // 2
        front_dist = msg.distances[mid]

        # Values are in cm
        if front_dist < 200:  # 2 meters
            self.front_clear = False
        else:
            self.front_clear = True

    def timer_callback(self):
        # Enable offboard control
        offboard = OffboardControlMode()
        offboard.velocity = True
        offboard.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_pub.publish(offboard)

        # Velocity setpoint
        sp = TrajectorySetpoint()
        sp.timestamp = offboard.timestamp

        if self.front_clear:
            sp.velocity = [1.0, 0.0, 0.0]   # forward (NED)
        else:
            sp.velocity = [0.0, 0.0, 0.0]   # STOP

        self.setpoint_pub.publish(sp)

        # Arm + switch to OFFBOARD (send a few times)
        if self.counter < 10:
            self.send_vehicle_command(176, 1.0)  # SET_MODE: OFFBOARD
            self.send_vehicle_command(400, 1.0)  # ARM
            self.counter += 1

    def send_vehicle_command(self, command, param1):
        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = param1
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = self.get_clock().now().nanoseconds // 1000
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

