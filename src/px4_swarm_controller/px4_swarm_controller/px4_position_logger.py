#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
import subprocess
import csv
import os
import shutil

class PX4PositionLogger(Node):
    def __init__(self):
        super().__init__('px4_position_logger')
        # self.get_logger().info("test1")
        self.declare_parameter('drone_naming', '')
        self.declare_parameter('drone_number', 0)
        (drone_naming, drone_number) = self.get_parameters(['drone_naming', 'drone_number'])

        # Replace with your actual namespace if needed (e.g., 'drone1')
        namespace = str(drone_naming.value) + str(drone_number.value)
        topic_name = f'{namespace}/fmu/out/vehicle_local_position_custom'

        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            topic_name,
            self.listener_callback,
            10
        )
        # self.get_logger().info("test2")

        if drone_number == 1:
            folder = "/home/del/drone_logs"
            for filename in os.listdir(folder):
                file_path = os.path.join(folder, filename)
                try:
                    if os.path.isfile(file_path) or os.path.islink(file_path):
                        os.unlink(file_path)
                    elif os.path.isdir(file_path):
                        shutil.rmtree(file_path)
                except Exception as e:
                    print('Failed to delete %s. Reason: %s' % (file_path, e))

        # Set up the CSV output file
        self.file_path = os.path.expanduser(f'~/drone_logs/{namespace}_local_position_log.csv')
        self.csv_file = open(self.file_path, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)

        # Write CSV header
        self.writer.writerow(['timestamp', 'x', 'y', 'z', 'vx', 'vy', 'vz'])
        # self.get_logger().info("test3")

    def listener_callback(self, msg: VehicleLocalPosition):
        # PX4 timestamps are uint64 in microseconds
        timestamp_sec = msg.timestamp * 1e-6
        self.writer.writerow([
            timestamp_sec,
            msg.x,
            msg.y,
            msg.z,
            msg.vx,
            msg.vy,
            msg.vz
        ])

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    logger = PX4PositionLogger()
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        subprocess.run("pkill gnome-terminal", shell=True)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
