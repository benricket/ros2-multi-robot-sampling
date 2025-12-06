#!/usr/bin/env python3
"""
ROS2 Node to send a start, pause, or resume signal to the simulation.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int32
from rclpy.qos import qos_profile_sensor_data
import termios
import tty
import sys

class SimController(Node):
    """
    Controller for pausing and resuming simulation via an int command
    """
    settings = termios.tcgetattr(sys.stdin)
    key = None

    def __init__(self):
        super().__init__('teleop_node')
        self.signal_pub = self.create_publisher(Int32, '/sim_control', 10)
        self.create_timer(0.01,self.run_loop)

    def run_loop(self):
        """
        Loop to run the control node
        """
        key = self.get_key()
        print(f"key = {key}")

        # Perform logic based on key identifier
        if key == ' ':
            print("Pausing simulation.")
            msg = Int32()
            msg.data = 0
            self.signal_pub.publish(msg)

        if key == '\n' or key == '\r':
            print("Resuming simulation.")
            msg = Int32()
            msg.data = 1
            self.signal_pub.publish(msg)

        # Stop program when Ctrl C pressed
        if key == '\x03':
            print("Stopping simulation.")
            msg = Int32()
            msg.data = 0
            self.signal_pub.publish(msg)
            raise KeyboardInterrupt
        
    def get_key(self):
        """
        Reads a keypress from the active terminal window
        
        Returns:
            key identifier for the next key entered in stdin
        """
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

def main():
    rclpy.init()
    node = SimController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

    
