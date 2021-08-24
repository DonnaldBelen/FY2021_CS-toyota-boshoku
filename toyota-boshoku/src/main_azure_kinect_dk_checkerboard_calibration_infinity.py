# -*- coding: utf-8 -*-
from subprocess import Popen, PIPE
from signal import signal, SIGINT
import time
import argparse

#arg
parser = argparse.ArgumentParser(description='azure kinect dk calibration')
parser.add_argument('--camera_no', type=int, default=0) # プレビュー
args = parser.parse_args()

running_procs = [
    Popen(['python3', './upload/upload_data.py'], bufsize=0),
    Popen(['python3', './azure_kinect_dk/checkerboard_calibration.py', '--infinity', '1', '--preview', '1', '--camera_no', str(args.camera_no)], bufsize=0),
    Popen(['./azure_kinect_dk/body_tracking', str(args.camera_no), '1', '1'], bufsize=0),
    ]

for proc in running_procs:
    proc.poll()

try:
    while True:
        time.sleep(.1)
except KeyboardInterrupt:
    Popen(['pkill', '-KILL', '-f', 'body_tracking'], bufsize=0).poll()
    for proc in running_procs:
        # proc.send_signal(SIGINT)
        # proc.communicate()
        proc.terminate()
