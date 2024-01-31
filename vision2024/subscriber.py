import socket
import struct

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            'tag_detections',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: AprilTagDetectionArray):
        tags = []
        for detection in msg.detections:
            pose: Pose = detection.pose.pose.pose
            tags.append((detection.id, pose))

        send_tcp(tags)

def send_tcp(tags: "list[tuple[int, Pose]]"):
    msg = struct.pack("!hi", 0x2175, len(tags))
    for id, pose in tags:
        tagStruct = struct.pack("!iddddddd", id, pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        msg = msg + tagStruct

    print(msg.hex())


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()