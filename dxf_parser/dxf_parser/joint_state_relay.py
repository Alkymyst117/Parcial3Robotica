import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import threading
import time

class JointStateRelay(Node):
    def __init__(self):
        super().__init__('joint_state_relay')
        self.sub = self.create_subscription(JointTrajectory, 'joint_trajectory', self.cb, 10)
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self._play_thread = None
        self._stop_event = threading.Event()

    def cb(self, msg: JointTrajectory):
        if self._play_thread is not None and self._play_thread.is_alive():
            self._stop_event.set()
            self._play_thread.join()

        self._stop_event.clear()
        self._play_thread = threading.Thread(target=self.play_trajectory, args=(msg,))
        self._play_thread.daemon = True
        self._play_thread.start()

    def play_trajectory(self, msg: JointTrajectory):
        slowdown_factor = 1.2
        
        last_t = 0.0
        for i, pt in enumerate(msg.points):
            if self._stop_event.is_set():
                return

            tsec = (pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9) * slowdown_factor
            sleep_dt = max(0.0, tsec - last_t)
            
            if sleep_dt > 0.0:
                time.sleep(sleep_dt)
                
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = msg.joint_names
            js.position = pt.positions
            if hasattr(pt, 'velocities') and pt.velocities:
                js.velocity = pt.velocities
            
            self.pub.publish(js)
            last_t = tsec
            
            if i % 50 == 0:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = JointStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
