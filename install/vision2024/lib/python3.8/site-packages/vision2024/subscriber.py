import wpilib
import ntcore
from wpimath.geometry import Pose3d, Translation3d, Rotation3d, Quaternion
import robotpy_apriltag

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray, AprilTagDetection

inst: ntcore.NetworkTableInstance = ntcore.NetworkTableInstance.getDefault()    
inst.startClient4("jetson")
inst.setServerTeam(2175)
inst.startDSClient()
inst.setServer("10.21.75.2", ntcore.NetworkTableInstance.kDefaultPort4)

poseTopic = inst.getStructArrayTopic("poses", Pose3d)

class MinimalSubscriber(Node):

    def __init__(self):
        self.tagPub = poseTopic.publish()
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            'tag_detections',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: AprilTagDetectionArray):
        self.tagPub.setDefault([])
        poses: list[Pose3d] = []
        ids: list[int] = []
        detection: AprilTagDetection
        for detection in msg.detections:
            pose: Pose = detection.pose.pose.pose
            poses.append(
                Pose3d(
                    Translation3d(pose.position.x, pose.position.y, pose.position.z),
                    Rotation3d(Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)),
                )
            )
            ids.append(detection.id)




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