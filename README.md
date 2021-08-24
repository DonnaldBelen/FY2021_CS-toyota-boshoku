# FY2021_CS-toyota-boshoku

#### Input/ Output [I/O] Data

1.) rosbag2_files

- Files used for simulating azure kinect camera data stream.



#### Docker

Toyota-boshoku/docker

- build and compile docker file.

- Ros2 Distro: foxy
- base os: Ubuntu20.04



#### ICP Repository / Json Calibration file creation





How to Run the source:

##### Terminal A:

```
export DISPLAY=:0
xhost +
```



##### Terminal B:

prepare environment

```
docker start <docker name>
docker attach <docker name>
source opt/ros/foxy/setup.bash
cd ~/share/toyota-boshoku/
source ros2_cs_module_install_rebuild.sh
```



#### launch src

##### on Terminal B:

```
python3 cs_control_pc.py
```



##### cs_control_pc.py -> details

```
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
```



##### purpose:

```
Popen(['ros2', 'run', 'cs_azure_kinect', 'capture', '0', '000060100112', '1', '0', '0', '30'], bufsize=0),
```

###### to run the installed azure kinect camera

```
Popen(['ros2', 'run', 'cs_detect_sitting_seat', 'run', '--no', '3'], bufsize=0),
```

###### To start/initialize the target node 'cs_detect_sitting_seat'

###### '3' -> corresponding seat number. points to config.ini ["seat_detection", "SEAT_3_POINT"]



##### ICP Repository for  calibration

```
--to update--
```





##### Main gesture algorithm

toyota-boshoku/src/ros2_cs/src/cs_detect_gesture/Detect_rule/DetectBodyStatus/...

