# -*- coding: utf-8 -*-
from subprocess import Popen, PIPE
from signal import signal, SIGINT
import time
import argparse

running_procs = [
    Popen(['ros2', 'run', 'cs_azure_kinect', 'capture', '0', '000060100112', '1', '0', '0', '30'], bufsize=0),
    # Popen(['ros2', 'run', 'cs_azure_kinect', 'capture', '1', '0', '0', '30'], bufsize=0),
    Popen(['ros2', 'run', 'cs_azure_kinect_merge', 'run', '--num', '2'], bufsize=0),

    #Popen(['ros2', 'run', 'cs_detect_sitting_seat', 'run', '--no', '1'], bufsize=0),
    #Popen(['ros2', 'run', 'cs_detect_sitting_seat', 'run', '--no', '2'], bufsize=0),
    Popen(['ros2', 'run', 'cs_detect_sitting_seat', 'run', '--no', '3'], bufsize=0),    

    #Popen(['ros2', 'run', 'cs_detect_action', 'run', '--no', '1'], bufsize=0),
    #Popen(['ros2', 'run', 'cs_detect_action', 'run', '--no', '2'], bufsize=0),
    #Popen(['ros2', 'run', 'cs_detect_action', 'run', '--no', '3'], bufsize=0),

    #Popen(['ros2', 'run', 'cs_detect_gesture', 'run', '--no', '1'], bufsize=0),
    #Popen(['ros2', 'run', 'cs_detect_gesture', 'run', '--no', '2'], bufsize=0),
    #Popen(['ros2', 'run', 'cs_detect_gesture', 'run', '--no', '3'], bufsize=0),
    
    #Popen(['ros2', 'run', 'cs_detect_section', 'run', '--no', '1'], bufsize=0),
    #Popen(['ros2', 'run', 'cs_detect_section', 'run', '--no', '2'], bufsize=0),
    #Popen(['ros2', 'run', 'cs_detect_section', 'run', '--no', '3'], bufsize=0),    
    
    #Popen(['ros2', 'run', 'cs_web_api', 'web_api'], bufsize=0),

    #Popen(['ros2', 'run', 'cs_ros2_to_websocket', 'run'], bufsize=0),
    ]

for proc in running_procs:
    proc.poll()

try:
    while True:
        time.sleep(.02)
except KeyboardInterrupt:
    for proc in running_procs:
        proc.send_signal(SIGINT)
        proc.communicate()
        proc.terminate()
