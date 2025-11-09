#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Quaternion

def yaw_from_quat(q: Quaternion) -> float:
    # Convert quaternion to yaw
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = -1.0 + 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def clamp(x, lo, hi): return max(lo, min(hi, x))

class StraightLineController(Node):
    def __init__(self):
        super().__init__('straight_line_controller')

     
        self.declare_parameter('goal_x', 10.0)
        self.declare_parameter('goal_y', 10.0)
        self.declare_parameter('kv', 0.4)        
        self.declare_parameter('kh', 0.6)          
        self.declare_parameter('max_v', 6.0)       
        self.declare_parameter('max_steer', 0.2)  
        self.declare_parameter('arrive_thresh', 0.20)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.kv = float(self.get_parameter('kv').value)
        self.kh = float(self.get_parameter('kh').value)
        self.max_v = float(self.get_parameter('max_v').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.arrive_thresh = float(self.get_parameter('arrive_thresh').value)

        # Publishers
        self.steer_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.vel_pub   = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # Odometry subscriber
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, qos)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.arrived = False

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_step)
        self.get_logger().info('StraightLineController running: goal=(%.2f, %.2f)' % (self.goal_x, self.goal_y))

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)

    def control_step(self):
        if self.arrived:
            self.publish_cmds(0.0, 0.0)
            return

        dx = (self.goal_x - self.x)
        dy = (self.goal_y - self.y)
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < self.arrive_thresh:
            self.arrived = True
            self.get_logger().info('Goal reached. Stopping.')
            self.publish_cmds(0.0, 0.0)
            return

        # Desired heading and heading error
        heading = math.atan2(dx, dy)
        print(heading)
        print(self.yaw)
        err = self.normalize_angle(heading - self.yaw)

        # Proportional controllers
        v = clamp(self.kv * dist, 0.0, self.max_v)            
        steer = clamp(self.kh * err, -self.max_steer, self.max_steer)  

        self.get_logger().info(f"pos=({self.x:.2f},{self.y:.2f}) d={dist:.2f} yaw={self.yaw:.2f} err={err:.2f} -> v={v:.2f}, steer={steer:.2f}")
        self.publish_cmds(v, steer)

    def publish_cmds(self, v: float, steer: float):
        vmsg = Float64MultiArray(); vmsg.data = [v, v]               # left, right
        smsg = Float64MultiArray(); smsg.data = [steer, steer]       # left, right steer
        self.vel_pub.publish(vmsg)
        self.steer_pub.publish(smsg)

    @staticmethod
    def normalize_angle(a):
        while a > math.pi: a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

def main(args=None):
    rclpy.init(args=args)
    node = StraightLineController()
    try:
        rclpy.spin(node)
    finally:
        node.publish_cmds(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
