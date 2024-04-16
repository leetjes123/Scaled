#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_ = self.create_publisher(Path, 'path', 10)

    def publish_path_from_csv(self, filename):
        path_msg = Path()
        with open(filename, 'r') as csvfile:
            csv_reader = csv.reader(csvfile)
            for row in csv_reader:
                if len(row) != 2:
                    self.get_logger().warn("Invalid row: %s", row)
                    continue
                pose = PoseStamped()
                pose.pose.position.x = float(row[0])
                pose.pose.position.y = float(row[1])
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)
        self.get_logger().info("Published path from file: %s", filename)

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    csv_filename = 'path_data.csv'  # Change this to your CSV file path
    path_publisher.publish_path_from_csv(csv_filename)
    rclpy.spin_once(path_publisher)  # Spin once to publish the path
    rclpy.shutdown()

if __name__ == '__main__':
    main()
