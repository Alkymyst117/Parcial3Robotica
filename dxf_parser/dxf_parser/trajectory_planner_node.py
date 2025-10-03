import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from scipy.interpolate import CubicSpline


class TrajectoryPlannerNode(Node):
    def __init__(self):
        super().__init__('trajectory_planner_node')
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'scara_joint_targets',
            self.listener_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            JointTrajectory,
            'joint_trajectory',
            10
        )
        
        self.waypoints = []
        self.joint_names = ['joint1', 'joint2', 'joint3']



    def listener_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 3:
            return
        
        waypoint = [msg.data[0], msg.data[1], msg.data[2]]
        self.waypoints.append(waypoint)
        
        if len(self.waypoints) >= 4:
            self.generate_trajectory()

    def generate_trajectory(self):
        waypoints = np.array(self.waypoints) 
        N = waypoints.shape[0]

        total_time = (N - 1) * 1.0
        t = np.linspace(0, total_time, N)
        
        points_per_second = 15
        total_points = int(total_time * points_per_second)
        t_new = np.linspace(0, total_time, total_points)
        
        q_interp = []
        for j in range(waypoints.shape[1]):
            cs = CubicSpline(t, waypoints[:, j])
            q_interp.append(cs(t_new))
        q_interp = np.array(q_interp).T
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        for i, ti in enumerate(t_new):
            point = JointTrajectoryPoint()
            point.positions = q_interp[i].tolist()

            if i > 0 and i < len(t_new) - 1:
                dt = t_new[i+1] - t_new[i-1]
                velocities = [(q_interp[i+1][j] - q_interp[i-1][j]) / dt for j in range(3)]
                point.velocities = velocities
            else:
                point.velocities = [0.0, 0.0, 0.0]
            
            point.time_from_start.sec = int(ti)
            point.time_from_start.nanosec = int((ti - int(ti)) * 1e9)
            traj.points.append(point)

        self.publisher.publish(traj)

        self.waypoints = []


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
