import csv
import re
import json
import numpy as np
import sys
import os
import configparser
import argparse
import time
import copy
import math
import traceback
import glob
import pprint
from pathlib import Path
import time
import cv2
import base64

import camera_data_merge

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../predict/')
import predict_json_getter
from datetime import date, datetime, timedelta

script_dir = os.path.dirname(os.path.abspath(__file__))

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

output_dir = "{}/{}".format(inifile.get('common', 'work_dir'), inifile.get('common', 'upload_dir'))
os.makedirs(output_dir, exist_ok=True)

IDX_EVENT_LOG_ID = 0
IDX_EVENT_LOG_NAME = 1
EVENT_NAME_AZURE_KINECT_CAPTURE = "AZURE_KINECT_CAPTURE"

json_getter = predict_json_getter.PredictJsonGetter()
json_getter.get_event_log() #一回読み捨て

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

def save_upload_data(data, sensor_type, sensor_no):
    sensing_time = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S.%f")

    frame_data = {
        "sensing_time": sensing_time,
        "type": sensor_type,
        "no": sensor_no,
        "data": data
    }

    # json出力
    save_json_file = "{}/{}_{}_{}".format(output_dir, sensing_time, sensor_type, sensor_no)
    with open(save_json_file, 'w') as f:
        json.dump(frame_data, f, cls = MyEncoder)
        print("Save...{}".format(save_json_file))
        f.close()
        os.rename(save_json_file, "{}.json".format(save_json_file))

    print("-----")
    # print(frame_data)

last_capture_time = 0
def check_capture_request():

    interval_mode = False
    if interval_mode:
        interval = 3000
        now = datetime.utcnow()
        elapsed_time = now - last_capture_time
        elapsed_seconds = elapsed_time.total_seconds()
        wait = interval - elapsed_seconds

        if elapsed_seconds > interval:
            elapsed_seconds = now
            return True:
        else:
            return False

    else:
        event_logs_json = json_getter.get_event_log()

        event_logs = json.loads(event_logs_json)

        for event_log in reversed(event_logs['data']):
            json_getter.last_event_log_id = event_log[IDX_EVENT_LOG_ID]
            event_name = event_log[IDX_EVENT_LOG_NAME]
            print("new evnet: {}".format(event_name))
            if event_name in [EVENT_NAME_AZURE_KINECT_CAPTURE]:
                print("AZURE_KINECT_CAPTURE: {}".format(event_name))
                return True

    return False

def check_capture_file():

    FACE_JOINT_NOS = [
        camera_data_merge.K4ABT_JOINT_NECK,
        camera_data_merge.K4ABT_JOINT_HEAD,
        camera_data_merge.K4ABT_JOINT_NOSE,
        camera_data_merge.K4ABT_JOINT_EYE_LEFT,
        camera_data_merge.K4ABT_JOINT_EAR_LEFT,
        camera_data_merge.K4ABT_JOINT_EYE_RIGHT,
        camera_data_merge.K4ABT_JOINT_EAR_RIGHT
    ]

    image_path = "{}/capture.png".format(script_dir)
    json_path = "{}/capture.json".format(script_dir)

    if not (os.path.exists(image_path) and os.path.exists(json_path)):
        return

    capture_image = cv2.imread(image_path)
    if capture_image is None:
        return

    with open(json_path) as f:
        sensor_data = json.load(f)

    data = sensor_data["data"]

    for psi in range(len(data["skeletons"])):

        # 顔範囲点
        x_list = []
        y_list = []

        for joint_no in FACE_JOINT_NOS:
            pos2d = np.array(data["skeletons"][psi]["joints"][joint_no]["position2d"])

            x_list.append(pos2d[0])
            y_list.append(pos2d[1])

        # 画像切り出し
        left = int(min(x_list))
        top = int(min(y_list))
        right = int(max(x_list))
        bottom = int(max(y_list))

        width = right - left
        height = bottom - top
    
        left -= width * 0.3
        right += width * 0.3
        top -= height * 1.2
        bottom += height * 0.3

        left = int(left)
        right = int(right)
        top = int(top)
        bottom = int(bottom)

        print("FACE              {}, {} - {}, {}".format(left, top, right , bottom))
        face_image = capture_image[top: bottom, left: right]
        image_file_path = "{}/face_{}.jpg".format(script_dir, data["skeletons"][psi]["id"])
        cv2.imwrite(image_file_path, face_image)

        enc_file = base64.encodestring(open(image_file_path, 'rb').read()).decode('utf8')

        save_upload_data({
            'photo' : enc_file,
            'skeleton' : data["skeletons"][psi]
        }, predict_json_getter.SENSOR_TYPE_FACE_PHOTO, data["skeletons"][psi]["id"])

    os.remove(image_path)

def main():
    print("処理開始")

    while True:
        try:
            if check_capture_request():
                Path("{}/capture.txt".format(script_dir)).touch()
                time.sleep(0.5)

            check_capture_file()
            
            time.sleep(0.08)
        except KeyboardInterrupt:
            break
        except:
            traceback.print_exc()

    print("処理完了")

if __name__ == "__main__":
    main()
    
