import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher_node')
        self.publisher_ = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 10)

        # Define multiple positions and orientations
        self.poses = [
            {
                'position': Point(x=81511.125, y=50193.7265625, z=0.0),
                'orientation': Quaternion(x=0.0, y=0.0, z=0.1452937536531552, w=0.989388561258607)
            },
            {
                'position': Point(x=81770.953125, y=50510.07421875, z=0.0),
                'orientation': Quaternion(x=0.0, y=0.0, z=-0.6259730256406028, w=0.7798447096508055)
            },
            {
                'position': Point(x=81756.640625, y=50545.6953125, z=0.0),
                'orientation': Quaternion(x=0.0, y=0.0, z=0.7732605964583231, w=0.6340883613227091)
            }
        ]

    def publish_pose(self, pose):
        msg = PoseStamped()

        # Set real-time timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Use the pose from the argument
        msg.pose.position = pose['position']
        msg.pose.orientation = pose['orientation']

        self.publisher_.publish(msg)
        self.get_logger().info('Published Pose')

def main(args=None):
    rclpy.init(args=args)

    pose_publisher = PosePublisher()

    print("按下回车键来发布消息...")
    for pose in pose_publisher.poses:
        input()  # Wait for enter key press
        pose_publisher.publish_pose(pose)

    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
