# -*- coding: utf-8 -*-
import configparser
import os
import shutil
from datetime import date, datetime, timedelta
import json
import time
import traceback
from PIL import Image, ImageDraw, ImageFilter
import glob
import re
import paho.mqtt.client as mqtt
import bson
import uuid
import threading
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../common/')
import cs_time_util

time_util = cs_time_util.CSTimeUtil()

AUTOWARE_TOPIC_LIST = [
    "autoware/filtered_points",
    "vehicle/pose",
    "vehicle/route",
    "vehicle/controls",
    "vehicle/state",
    "vehicle/cmd"
]

# 設定読み込み
inifile = configparser.ConfigParser()
inifile_dir = os.path.dirname(os.path.abspath(__file__))
inifile_path = "%s/config.ini" % inifile_dir
for i in range(10):
    if os.path.exists(inifile_path):
        break
    else:
        inifile_dir = "%s/.." % inifile_dir
        inifile_path = "%s/config.ini" % inifile_dir
print("load %s" % inifile_path)
inifile.read(inifile_path, 'UTF-8')

mqtt_client = None

def mqtt_pulish():
    try:
        ride_vision_data = {
            'timestamp': time_util.get_time(),
            'data' : {
                'hoge' : 123,
                'tx' : 'string'
            }
        }

        bson_data = bson.dumps(ride_vision_data)
        mqtt_client.publish(AUTOWARE_TOPIC_LIST[0], bson_data, 2)
    except:
        traceback.print_exc()

def mqtt_on_connect(client, userdata, flags, respons_code):
    """broker接続時のcallback関数
    """
    print('mqtt_on_connect status {0}'.format(respons_code))
    client.subscribe('#')

    mqtt_pulish()

def mqtt_on_message(client, userdata, msg):
    """メッセージ受信時のcallback関数
    """

    try:        
        print("mqtt_on_message:" + msg.topic + ' ' + str(msg.payload)) 

        data = bson.loads(msg.payload)        
        print(data)
    except:
        traceback.print_exc()

if __name__ == '__main__':
    if inifile.has_option('common', 'mqtt_address'):
        mqtt_address = inifile.get('common', 'mqtt_address')
        if len(mqtt_address) > 0:
            mqtt_addresss = mqtt_address.split(":")
            mqtt_ip = mqtt_addresss[0]
            mqtt_port = mqtt_addresss[1] if len(mqtt_addresss) >= 2 else 1883

            mqtt_client = mqtt.Client()

            mqtt_client.on_connect = mqtt_on_connect
            mqtt_client.on_message = mqtt_on_message

            mqtt_client.connect(mqtt_ip, int(mqtt_port), keepalive=60)

            def mqtt_loop():
                global mqtt_client
                mqtt_client.loop_forever()

            mqtt_thread = threading.Thread(target=mqtt_loop)
            mqtt_thread.start()