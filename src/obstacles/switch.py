#!/usr/bin/python3
import subprocess
import signal
import rclpy
from std_msgs.msg import Bool
from rclpy.node import Node
import time

# Global variables to keep track of subprocesses
detector_process = None
initpoint_process = None
final_process = None
initial_process = None
ahead_process = None
aligner_process = None

def start_detector():
    global detector_process
    if detector_process is None:
        detector_process = subprocess.Popen(['ros2', 'run', 'lanefollowing', 'detector'])
        time.sleep(3)

def start_initpoint():
    global initpoint_process
    if initpoint_process is None: # and is_topic_publishing("/gray_image_topic"):
        initpoint_process = subprocess.Popen(['ros2', 'run', 'lanefollowing', 'initpoint'])
        time.sleep(3)

def start_final():
    global final_process
    if final_process is None: # and is_topic_publishing("/init_left"):
        final_process = subprocess.Popen(['ros2', 'run', 'lanefollowing', 'final'])

def start_initial():
    global initial_process
    if initial_process is None:
        initial_process = subprocess.Popen(['ros2', 'run', 'lanefollowing', 'initial.py'])

def start_ahead():
    global ahead_process
    if ahead_process is None: 
        ahead_process = subprocess.Popen(['ros2', 'run', 'lanefollowing', 'ahead.py'])

def start_aligner():
    global aligner_process
    if aligner_process is None: 
        aligner_process = subprocess.Popen(['ros2', 'run', 'lanefollowing', 'aligner.py'])

def stop_process(process):
    if process:
        process.send_signal(signal.SIGINT)
        process.wait()

# def is_topic_publishing(topic_name):
#     output = subprocess.run(['ros2', 'topic', 'echo', topic_name, '-n', '1'], capture_output=True)
#     return output.returncode == 0

def avoid_callback(msg):
    global detector_process, initpoint_process, final_process, initial_process

    if msg.data:  # If data is True
        print("Avoiding obstacle, lane following on hold")
        # Kill all running nodes of lanefollowing
        # stop_process(detector_process)
        # stop_process(initpoint_process)
        # stop_process(final_process)

        subprocess.run(['pkill','detector'])
        subprocess.run(['pkill','initpoint'])
        subprocess.run(['pkill','final'])
        
        detector_process = None
        initpoint_process = None
        final_process = None
        time.sleep(2)
        # Start ahead.py
        try:
            start_initial()
            # start_ahead()
            # subprocess.run(['ros2', 'run', 'lanefollowing', 'initial.py', '&&', 'ros2', 'run', 'lanefollowing', 'ahead.py'])
            subprocess.run(['ros2', 'run', 'lanefollowing', 'ahead.py'])
        except Exception as e:
            print("Error running ahead.py:", e)
        
    else:  # If data is False
        print("Sticking to Lanefollowing")
        if initial_process is not None : 
            subprocess.run(['ros2', 'run', 'lanefollowing', 'aligner.py'])
            # start_aligner()
        subprocess.run(['pkill','initial_angz'])
        initial_process = None
        # Start detector
        start_detector()
        # Start initpoint if /gray_image_topic is publishing
        start_initpoint()
        # Start final if /init_left is publishing
        start_final()

class Switch(Node):
    def __init__(self):
        super().__init__('Switch')
        self.subscription = self.create_subscription(Bool, '/lanefollowing/avoid', self.avoid_callback, 10)

    def avoid_callback(self, msg):
        avoid_callback(msg)

def main():
    rclpy.init()
    node = Switch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
