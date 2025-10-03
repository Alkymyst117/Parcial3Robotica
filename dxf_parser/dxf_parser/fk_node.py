import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import PoseStamped
import math


class FKNode(Node):
    def __init__(self):
        super().__init__('fk_node')
        
        self.subscription = self.create_subscription(
            JointTrajectory,
            'joint_trajectory',
            self.trajectory_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            PoseStamped,
            'scara_fk_pose',
            10
        )
        
        self.L1 = 0.1445
        self.L2 = 0.1445



    def trajectory_callback(self, msg: JointTrajectory):
        if not msg.points:
            return
        
        for point in msg.points:
            q1, q2, s3 = point.positions
            
            x = self.L1 * math.cos(q1) + self.L2 * math.cos(q1 + q2)
            y = self.L1 * math.sin(q1) + self.L2 * math.sin(q1 + q2)
            z = s3
            
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            
            self.publisher.publish(pose)



def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
