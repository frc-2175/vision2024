import rclpy
from rclpy.node import Node

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
        if len(msg.detections) != 0:
            self.get_logger().info('Tag %d: (%f, %f, %f)' % msg.detections[0].id, msg.detections[0].center.x, msg.detections[0].center.y, msg.detections[0].center.z)


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