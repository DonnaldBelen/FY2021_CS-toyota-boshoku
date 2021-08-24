import rclpy
import configparser
import time
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
import traceback
import glob
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/')
from cs_common.csdds import CSSubscriberNode
from cs_common.csdds import CSPublisherNode
from cs_common.csdds import CSQoS
from rclpy.executors import MultiThreadedExecutor
import subprocess
import json
from datetime import date, datetime, timedelta
import re
import yaml
import numpy as np

SENSOR_TYPE_ACC = '4'
SENSOR_TYPE_ACC_ESTIMATE = '8'

SENSOR_TYPE_RESPIRATION = '20'
SENSOR_TYPE_WHS2 = '21'
SENSOR_TYPE_RESPIRATION_RAW = '24'
SENSOR_TYPE_CONCENTRATE = '26'
SENSOR_TYPE_USER_CALIBRATION = '27'
SENSOR_TYPE_RESPIRATION_FEAUTURE = '29'

SENSOR_TYPE_TOBII_PRO = '34'
SENSOR_TYPE_RECORD = '35'

SENSOR_TYPE_HEAD_MOVING = '13'
SENSOR_TYPE_HAND_GESTURE = '46'
SENSOR_TYPE_HAND_POINTER = '48'
SENSOR_TYPE_LIKE_SIGN = '49'
SENSOR_TYPE_LIGHT = '41'

SENSOR_TYPE_RIDEVISION_SCENE = '60'

SENSOR_TYPE_AZURE_KINECT_DK = '71'
SENSOR_TYPE_AZURE_KINECT_DK_CALIBRATION = '72'
SENSOR_TYPE_AZURE_KINECT_DK_CHECKERBOARD_CALIBRATION = '73'

SENSOR_TYPE_FRAGRANCE = '76'
SENSOR_TYPE_DMX = '77'
SENSOR_TYPE_PERSON_POSITION = '78'
SENSOR_TYPE_SET_SOUND_VOLUME = '79'

SENSOR_TYPE_FACE_PHOTO = '80'
SENSOR_TYPE_REACTION = '47'

SENSOR_TYPE_BRAINVU = '70'
SENSOR_TYPE_ACTION_RECOG = '81'
SENSOR_TYPE_SET_SOUND_VOLUME_INDICATOR = '85'
SENSOR_TYPE_SET_VIB_VOLUME_INDICATOR = '86'
SENSOR_TYPE_SET_FRAGRANCE_INDICATOR = '87'
SENSOR_TYPE_SET_LIGHT_INDICATOR = '88'
SENSOR_TYPE_BRANCH_LOG = '89'
SENSOR_TYPE_QUESTIONNAIRE_DATA_LOG = '90'

K4ABT_JOINT_PELVIS = 0
K4ABT_JOINT_SPINE_NAVAL = 1
K4ABT_JOINT_SPINE_CHEST = 2
K4ABT_JOINT_NECK = 3
K4ABT_JOINT_CLAVICLE_LEFT = 4
K4ABT_JOINT_SHOULDER_LEFT = 5
K4ABT_JOINT_ELBOW_LEFT = 6
K4ABT_JOINT_WRIST_LEFT = 7
K4ABT_JOINT_HAND_LEFT = 8
K4ABT_JOINT_HANDTIP_LEFT = 9
K4ABT_JOINT_THUMB_LEFT = 10
K4ABT_JOINT_CLAVICLE_RIGHT = 11
K4ABT_JOINT_SHOULDER_RIGHT = 12
K4ABT_JOINT_ELBOW_RIGHT = 13
K4ABT_JOINT_WRIST_RIGHT = 14
K4ABT_JOINT_HAND_RIGHT = 15
K4ABT_JOINT_HANDTIP_RIGHT = 16
K4ABT_JOINT_THUMB_RIGHT = 17
K4ABT_JOINT_HIP_LEFT = 18
K4ABT_JOINT_KNEE_LEFT = 19
K4ABT_JOINT_ANKLE_LEFT = 20
K4ABT_JOINT_FOOT_LEFT = 21
K4ABT_JOINT_HIP_RIGHT = 22
K4ABT_JOINT_KNEE_RIGHT = 23
K4ABT_JOINT_ANKLE_RIGHT = 24
K4ABT_JOINT_FOOT_RIGHT = 25
K4ABT_JOINT_HEAD = 26
K4ABT_JOINT_NOSE = 27
K4ABT_JOINT_EYE_LEFT = 28
K4ABT_JOINT_EAR_LEFT = 29
K4ABT_JOINT_EYE_RIGHT = 30
K4ABT_JOINT_EAR_RIGHT = 31

class MyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        else:
            return super(MyEncoder, self).default(obj)

def support_datetime_default(o):
    if isinstance(o, datetime):
        return o.strftime("%Y-%m-%d %H:%M:%S")
    if isinstance(o, Decimal):
        return float(o)
    raise TypeError(repr(o) + " is not JSON serializable")

def call_ros2_service(service_name, msg_type, msg):
    msg_yaml = yaml.dump(msg)

    # ros2 service call /template_service cs_interfaces/srv/AddTwoInts '{a: 1,b: 2}'
    proc = subprocess.Popen(['ros2', 'service', 'call', service_name, msg_type, msg_yaml], stdout=subprocess.PIPE)
    out, err = proc.communicate()
    print(out.decode('utf-8'))

    return proc.returncode

def call_ros2_service_by_dic(service_name, msg_type, msg_dic):
    data_str = json.dumps(msg_dic, default=support_datetime_default)
    call_ros2_service(service_name, msg_type, {'data': data_str})

def load_ini_file():
    inifile = configparser.ConfigParser()
    inifile_dir = os.path.dirname(os.path.abspath(__file__))
    inifile_path = "%s/cs_config.ini" % inifile_dir
    for i in range(10):
        if os.path.exists(inifile_path):
            break
        else:
            inifile_dir = "%s/.." % inifile_dir
            inifile_path = "%s/cs_config.ini" % inifile_dir
    print("load %s" % inifile_path)
    inifile.read(inifile_path, 'UTF-8')

    return inifile


class CSNode(Node):
    def __init__(self, name):
        super().__init__(name)

        # 設定読み込み
        self.inifile = load_ini_file()
        self.pubnode = {}

    def publish_data(self, topic, frame_data, sensing_time = None):

        if 'sensing_time' not in frame_data:
            if sensing_time is None:
                sensing_time = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S.%f")

            frame_data['sensing_time'] = sensing_time

        json_str = json.dumps(frame_data, cls = MyEncoder)

        if topic not in self.pubnode:
            qos_profile = CSQoS.create(CSQoS.QoSType.SENSOR)            
            self.pubnode[topic] = CSPublisherNode(topic, String, qos_profile)

        self.pubnode[topic].publish(json_str)