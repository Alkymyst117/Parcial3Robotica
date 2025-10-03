import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray
import math


class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        
        self.subscription = self.create_subscription(
            PointStamped,
            'scara_waypoints',
            self.listener_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'scara_joint_targets',
            10
        )
        
        self.L1 = 144.5
        self.L2 = 144.5
        self.min_theta1 = -62.0
        self.max_theta1 = 62.0
        self.min_theta2 = -130.0
        self.max_theta2 = 130.0
        self.min_z_mm = 0.0
        self.max_z_mm = 15.0



    def listener_callback(self, msg: PointStamped):
        x_mm = msg.point.x
        y_mm = msg.point.y
        z_mm = msg.point.z
        
        if z_mm < self.min_z_mm or z_mm > self.max_z_mm:
            return
        
        x = x_mm / 1000.0
        y = y_mm / 1000.0
        z = -z_mm / 1000.0
        L1 = self.L1 / 1000.0
        L2 = self.L2 / 1000.0
        
        r = math.hypot(x, y)
        if r > (L1 + L2) or r < abs(L1 - L2):
            return
        
        cos_theta2 = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2)
        cos_theta2 = max(min(cos_theta2, 1.0), -1.0)
        
        theta2_options = [math.acos(cos_theta2), -math.acos(cos_theta2)]
        valid_solutions = []
        
        for theta2 in theta2_options:
            theta1 = math.atan2(y, x) - math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
            theta1_deg = math.degrees(theta1)
            theta2_deg = math.degrees(theta2)
            
            if self.is_within_limits(theta1_deg, theta2_deg):
                valid_solutions.append((theta1, theta2))
        
        if not valid_solutions:
            return
        
        chosen = min(valid_solutions, key=lambda s: abs(s[0]) + abs(s[1]))
        theta1, theta2 = chosen
        s3 = z
        
        out = Float32MultiArray()
        out.data = [theta1, theta2, s3]
        self.publisher.publish(out)

    def is_within_limits(self, theta1_deg, theta2_deg):
        return (self.min_theta1 <= theta1_deg <= self.max_theta1) and \
               (self.min_theta2 <= theta2_deg <= self.max_theta2)


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
