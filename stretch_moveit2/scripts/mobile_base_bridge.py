#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist

class PlanarBridge(Node):
    def __init__(self):
        super().__init__('planar_bridge')
        self.sub = self.create_subscription(
            JointTrajectory,
            '/mobile_base_controller/joint_trajectory',
            self.cb, 10)
        self.pub = self.create_publisher(
            Twist,
            '/mobile_base_dd_controller/cmd_vel_unstamped',
            10)

    def cb(self, traj: JointTrajectory):
        if not traj.points:
            return
        p = traj.points[0]
        dt = p.time_from_start.sec + p.time_from_start.nanosec*1e-9
        if dt <= 0.0 or len(p.positions) < 3:
            return
        x, y, th = p.positions
        vx, vy, w = x/dt, y/dt, th/dt
        t = Twist()
        t.linear.x  = vx
        t.linear.y  = vy      # diff-drive ignores this
        t.angular.z = w
        self.pub.publish(t)

def main():
    rclpy.init()
    rclpy.spin(PlanarBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
