import dataclasses
import ntcore
from wpimath.geometry import Pose3d, Translation3d, Rotation3d, Quaternion, Pose2d, Transform2d, Transform3d

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray, AprilTagDetection

import robotpy_apriltag
import wpiutil
import wpiutil.wpistruct

@wpiutil.wpistruct.make_wpistruct(name="tagdetection")
@dataclasses.dataclass
class TagDetection:
    id: int
    transform: Transform3d

    def __init__(self, id: int, transform: Transform3d) -> None:
        self.id = id
        self.transform = transform

inst: ntcore.NetworkTableInstance = ntcore.NetworkTableInstance.getDefault()    
inst.startClient4("jetson")
inst.setServerTeam(2175)
inst.startDSClient()
inst.setServer("192.168.55.100", ntcore.NetworkTableInstance.kDefaultPort4)

table = inst.getTable("jetson")

testPoseTopic = table.getStructArrayTopic("detections", TagDetection)

class MinimalSubscriber(Node):
    def __init__(self, ):
        self.testPub = testPoseTopic.publish()
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            'tag_detections',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg: AprilTagDetectionArray):
        self.testPub.setDefault([])
        poses: list[TagDetection] = []
        detection: AprilTagDetection
        for detection in msg.detections:
            pose: Pose = detection.pose.pose.pose

            # wpilibPose = Transform3d(pose.position.z, -pose.position.x, pose.position.y, Rotation3d(Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)))
            wpilibPose = Transform3d(pose.position.x, pose.position.y, pose.position.z, Rotation3d(Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)))

            print(wpilibPose)
            poses.append(TagDetection(detection.id, wpilibPose))
            
        self.testPub.set(poses)




def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    
    print("darn")
	
    rclpy.shutdown()


if __name__ == '__main__':
    main()
