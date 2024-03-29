#!/usr/bin/python3
import subprocess
import rclpy
from std_msgs.msg import Bool

def avoid_callback(msg):
    if msg.data:  # If data is True
        print("Avoiding obstacle, lane following on hold")
        try:
            # Execute ahead.py using subprocess
            subprocess.run(['python3', 'src/lanefollowing/src/ahead.py'])
        except Exception as e:
            print("Error running ahead.py:", e)
        
    else :
        print("Sticking to Lanefollowing")


def main():
    rclpy.init()
    node = rclpy.create_node('avoid_listener')
    subscriber = node.create_subscription(Bool, '/lanefollowing/avoid', avoid_callback, 10)
    subscriber  # Prevent unused variable warning
    rclpy.spin(node)

if __name__ == '__main__':
    main()
