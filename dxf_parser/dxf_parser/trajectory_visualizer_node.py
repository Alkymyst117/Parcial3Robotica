import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math

class TrajectoryVisualizerNode(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer_node')
        
        self.subscription = self.create_subscription(
            PointStamped,
            'scara_waypoints',
            self.waypoint_callback,
            10
        )
        
        self.marker_pub = self.create_publisher(MarkerArray, 'trajectory_markers', 10)
        
        self.trajectory_points = []
        self.marker_id = 0
        
        self.cut_line_width = 0.001
        self.travel_line_width = 0.0005
        
        self.cut_color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0)
        self.travel_color = ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.4)

    def waypoint_callback(self, msg: PointStamped):
        x = msg.point.x / 1000.0
        y = msg.point.y / 1000.0
        z = msg.point.z / 1000.0
        
        self.trajectory_points.append((x, y, z))
        self.create_trajectory_markers()
        
        self.get_logger().debug(f'Punto agregado a trayectoria: ({x:.3f}, {y:.3f}, {z:.3f})')

    def create_trajectory_markers(self):
        marker_array = MarkerArray()
        
        if len(self.trajectory_points) < 2:
            return
        
        for i in range(len(self.trajectory_points) - 1):
            p1 = self.trajectory_points[i]
            p2 = self.trajectory_points[i + 1]
            
            z1_mm = p1[2] * 1000 
            z2_mm = p2[2] * 1000  
            
            if z1_mm <= 1.0 and z2_mm <= 1.0:  
                cut_marker = self.create_line_marker(p1, p2, self.cut_color, self.cut_line_width, i)
                marker_array.markers.append(cut_marker)
                
            else: 
                travel_marker = self.create_line_marker(p1, p2, self.travel_color, self.travel_line_width, i)
                marker_array.markers.append(travel_marker)
        
        self.marker_pub.publish(marker_array)

    def create_line_marker(self, p1, p2, color, width, marker_id):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory_lines"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.scale.x = width
        marker.color = color
        
        from geometry_msgs.msg import Point
        point1 = Point()
        point1.x, point1.y = p1[0], p1[1]
        point1.z = 0.001 
        
        point2 = Point()
        point2.x, point2.y = p2[0], p2[1] 
        point2.z = 0.001  
        
        marker.points = [point1, point2]
        marker.lifetime.sec = 0  
        
        return marker

    def clear_trajectory(self):
        marker_array = MarkerArray()
        
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        self.marker_pub.publish(marker_array)
        self.trajectory_points.clear()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()