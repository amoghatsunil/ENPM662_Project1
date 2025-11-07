#!/usr/bin/env python3
import sys, select, tty, termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

VEL_STEP = 1.0     # wheel speed step per keypress (e.g., rad/s)
STEER_STEP = 0.05  # steering step per keypress (rad)
MAX_VEL = 30.0     # clamp to your controller's safe range
MAX_STEER = 0.6    # ~34 degrees; adjust to your joint limits

def clamp(x, lo, hi): return max(lo, min(hi, x))

HELP = """
Keyboard Drive
--------------------------------------------------
w/s : faster/slower (both wheels)
a/d : steer left/right (both front steer joints)
q   : stop (zero velocity & center steering)
Esc : quit
"""

class Teleop2x2(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')

        # Topics match your commands:
        self.steer_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.vel_pub   = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        self.v = 0.0
        self.steer = 0.0

        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd)
        self.get_logger().info(HELP)

    def get_key(self, t=0.1):
        tty.setraw(self.fd)
        try:
            r,_,_ = select.select([sys.stdin], [], [], t)
            return sys.stdin.read(1) if r else ''
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def publish(self):
        # velocities: [left, right] (swap if your robot is reversed)
        vmsg = Float64MultiArray()
        vmsg.data = [self.v, self.v]
        self.vel_pub.publish(vmsg)

        # steering: [left, right]
        smsg = Float64MultiArray()
        smsg.data = [self.steer, self.steer]
        self.steer_pub.publish(smsg)

    def stop(self):
        self.v = 0.0
        self.steer = 0.0
        self.publish()

    def run(self):
        try:
            while rclpy.ok():
                k = self.get_key()
                if k == '\x1b':  # ESC
                    self.stop(); break
                elif k == 'q':
                    self.stop(); self.get_logger().info("Force stop.")
                elif k == 'w':
                    self.v = clamp(self.v + VEL_STEP, -MAX_VEL, MAX_VEL)
                elif k == 's':
                    self.v = clamp(self.v - VEL_STEP, -MAX_VEL, MAX_VEL)
                elif k == 'a':
                    self.steer = clamp(self.steer + STEER_STEP, -MAX_STEER, MAX_STEER)
                elif k == 'd':
                    self.steer = clamp(self.steer - STEER_STEP, -MAX_STEER, MAX_STEER)
                elif k == '':
                    continue
                self.get_logger().info(f"wheel_v={self.v:.2f}, steer={self.steer:.2f} rad")
                self.publish()
        finally:
            try: termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
            except Exception: pass
            self.stop()

def main(args=None):
    rclpy.init(args=args)
    node = Teleop2x2()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
