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

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../predict/')
import predict_json_getter
from datetime import date, datetime, timedelta

import urllib.request, urllib.parse
import socket

# K4ABT_JOINT_PELVIS = 0,
# K4ABT_JOINT_SPINE_NAVAL = 1
# K4ABT_JOINT_SPINE_CHEST = 2
# K4ABT_JOINT_NECK = 3
# K4ABT_JOINT_CLAVICLE_LEFT = 4
# K4ABT_JOINT_SHOULDER_LEFT = 5
# K4ABT_JOINT_ELBOW_LEFT = 6
# K4ABT_JOINT_WRIST_LEFT = 7
# K4ABT_JOINT_CLAVICLE_RIGHT = 8
# K4ABT_JOINT_SHOULDER_RIGHT = 9
# K4ABT_JOINT_ELBOW_RIGHT = 10
# K4ABT_JOINT_WRIST_RIGHT = 11
# K4ABT_JOINT_HIP_LEFT = 12
# K4ABT_JOINT_KNEE_LEFT = 13
# K4ABT_JOINT_ANKLE_LEFT = 14
# K4ABT_JOINT_FOOT_LEFT = 15
# K4ABT_JOINT_HIP_RIGHT = 16
# K4ABT_JOINT_KNEE_RIGHT = 17
# K4ABT_JOINT_ANKLE_RIGHT = 18
# K4ABT_JOINT_FOOT_RIGHT = 19
# K4ABT_JOINT_HEAD = 20
# K4ABT_JOINT_NOSE = 21
# K4ABT_JOINT_EYE_LEFT = 22
# K4ABT_JOINT_EAR_LEFT = 23
# K4ABT_JOINT_EYE_RIGHT = 24
# K4ABT_JOINT_EAR_RIGHT = 25

#0.9.4
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

use_api = True
pre_last_id = None
merge_mode = 2

# merge_mode : 1
# 人間キャリブ
# merge_mode : 2
# チェッカーキャリブ
# merge_mode : 3
# 原点チェッカー
# 回転チェッカー


SENSOR_NO_MOOX = '98'
SENSOR_NO_MERGE = '99'
SENSOR_NO_LIST = ['0', '1']

IDX_EVENT_LOG_ID = 0
IDX_EVENT_LOG_NAME = 1
EVENT_NAME_NUITRACK_CALIBRATION = "NUITRACK_CALIBRATION"
EVENT_NAME_NUITRACK_CALIBRATION_MORE_ACCURATE = "NUITRACK_CALIBRATION_MORE_ACCURATE"

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

moox_api_address = inifile.get('common', 'moox_api_address')

# キャリブレーションファイル
NUITRACK_CALIBRATION_FILENAME = "{}/../nuitrack_calibration.json".format(os.path.dirname(os.path.abspath(__file__)))

# arg
parser = argparse.ArgumentParser(description='azure kinect dk calibration')
parser.add_argument('--calibration', type=int, default=0) # プレビュー
args = parser.parse_args()

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

def get_camera_data_by_sql():
    sql_path = os.path.dirname(os.path.abspath(__file__)) + "/sensor_data.sql"
    camera1_datas, camera2_datas = read_sql_file(sql_path)

    return True, 1, camera1_datas, camera2_datas

def get_camera_data_by_api():
    api_predict_json_str = json_getter.get(1, {
        predict_json_getter.SENSOR_TYPE_AZURE_KINECT_DK : SENSOR_NO_LIST,
        predict_json_getter.SENSOR_TYPE_AZURE_KINECT_DK_CALIBRATION : [SENSOR_NO_MERGE],
        predict_json_getter.SENSOR_TYPE_AZURE_KINECT_DK_CHECKERBOARD_CALIBRATION : SENSOR_NO_LIST
    })

    if api_predict_json_str is None:
        return False, None, None, None, None

    api_predict_json = json.loads(api_predict_json_str)

    predict_json_data = api_predict_json["data"]

    camera_data_list = []
    checkerboard_data_list = []
    current_last_id = None
    try:

        for i in SENSOR_NO_LIST:
            camera_data = predict_json_data[predict_json_getter.SENSOR_TYPE_AZURE_KINECT_DK][i][0][predict_json_getter.IDX_SENSOR_DATA]
            camera_data_dict = json.loads(camera_data)
            if camera_data_dict:
                skeletons = camera_data_dict["skeletons"]
                for skeleton in skeletons:
                    skeleton["sensor_no"] = i
            camera_data_list.append(camera_data_dict)

            current_id = predict_json_data[predict_json_getter.SENSOR_TYPE_AZURE_KINECT_DK][i][0][predict_json_getter.IDX_SENSOR_ID]
            if current_last_id is None or current_last_id < current_id:
                current_last_id = current_id

            if merge_mode == 2 or merge_mode == 3:
                checkerboard_data = predict_json_data[predict_json_getter.SENSOR_TYPE_AZURE_KINECT_DK_CHECKERBOARD_CALIBRATION][i][0][predict_json_getter.IDX_SENSOR_DATA]
                checkerboard_data_dict = json.loads(checkerboard_data)
                checkerboard_data_list.append(checkerboard_data_dict)
            else:
                checkerboard_data_list.append([])

        calibration_data = None
        if len(predict_json_data[predict_json_getter.SENSOR_TYPE_AZURE_KINECT_DK_CALIBRATION][SENSOR_NO_MERGE]) > 0:
            calibration_data_str = predict_json_data[predict_json_getter.SENSOR_TYPE_AZURE_KINECT_DK_CALIBRATION][SENSOR_NO_MERGE][0][predict_json_getter.IDX_SENSOR_DATA]
            calibration_data = json.loads(calibration_data_str)

    except:
        traceback.print_exc()
        return False, None, None, None, None

    return True, current_last_id, camera_data_list, calibration_data, checkerboard_data_list

def get_camera_data():
    if use_api:
        return get_camera_data_by_api()
    else:
        return get_camera_data_by_sql()

def is_update_id(current_last_id):
    global pre_last_id
    if pre_last_id is not None and current_last_id == pre_last_id:
        print("カメラデータが更新されていません。pre_last_id:{}/current_last_id:{}".format(pre_last_id, current_last_id))
        return False
    else:
        pre_last_id = current_last_id
        return True

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

    # print("-----")
    # print(frame_data)

def run_calibration(camera_data_list):
    calibration_data = get_dif_moox_coordinate(camera_data_list)
    if calibration_data == None:
        print("get_dif_moox_coordinate 失敗")
        return None

    # キャリブレーション値保存
    save_upload_data(calibration_data, predict_json_getter.SENSOR_TYPE_AZURE_KINECT_DK_CALIBRATION, SENSOR_NO_MERGE)

    return calibration_data

def check_calibration_request():
    event_logs_json = json_getter.get_event_log()

    event_logs = json.loads(event_logs_json)

    for event_log in reversed(event_logs['data']):
        json_getter.last_event_log_id = event_log[IDX_EVENT_LOG_ID]
        event_name = event_log[IDX_EVENT_LOG_NAME]
        print("new evnet: {}".format(event_name))
        if event_name in [EVENT_NAME_NUITRACK_CALIBRATION, EVENT_NAME_NUITRACK_CALIBRATION_MORE_ACCURATE]:
            print("CALIBRATION　: {}".format(event_name))
            return event_name

    return False

def main():
    print("処理開始")

    # キャリブレーション値生成
    ret, current_last_id, camera_data_list, calibration_data, checkerboard_data_list = get_camera_data()
    print("calibration {}".format(args.calibration))
    if args.calibration >= 1:
        calibration_data = run_calibration(camera_data_list)

    #マージデータ表示
    pre_merged_data = None
    while True:
        try:

            ret, current_last_id, camera_data_list, calibration_data, checkerboard_data_list = get_camera_data()

            # apiデータがアップデートされているか判定
            if not is_update_id(current_last_id):
                time.sleep(0.05)
                continue

            #キャリブレーション
            if merge_mode == 2 or merge_mode == 3:
                if len(camera_data_list) != len(checkerboard_data_list):
                    print("チェッカーボード　キャリブレーションデータ無し")
                    continue

            if merge_mode == 1 or merge_mode == 3:
                calibration_request_mode = check_calibration_request()

                if calibration_data is None \
                    or len(camera_data_list) != len(calibration_data) \
                    or calibration_request_mode == EVENT_NAME_NUITRACK_CALIBRATION:
                    calibration_data = run_calibration(camera_data_list)

                if calibration_data is None:
                    print("NUITRACK キャリブレーションデータ無し")
                    continue

            # moox座標系に変換
            moox_data_list = []
            for i in range(len(camera_data_list)):

                i_calibration = None
                if calibration_data is not None and len(calibration_data) > i:
                    i_calibration = calibration_data[i]

                i_checkerboard = None
                if checkerboard_data_list is not None and len(checkerboard_data_list) > i:
                    i_checkerboard = checkerboard_data_list[i]

                moox_datas = get_moox_data(camera_data_list[i], i_calibration, i_checkerboard)
                moox_data_list.append(moox_datas)

            # キャリブレーション値補正処理
            if merge_mode == 1 or merge_mode == 3:
                if calibration_request_mode == EVENT_NAME_NUITRACK_CALIBRATION_MORE_ACCURATE:
                    calibration_accurate(camera_data_list, moox_data_list, calibration_data)

            # 変換した座標をマージ
            merged_data = get_merged_data(moox_data_list, pre_merged_data)
            pre_merged_data = copy.deepcopy(merged_data)
            pre_merged_data = copy.deepcopy(merged_data)

            # キャプチャが必要か判断
            # if check_need_capture(merged_data):
            #     call_moox_api('write_event_log', {"name" : "AZURE_KINECT_CAPTURE"})


            check_person_position(merged_data)

            save_upload_data(merged_data, predict_json_getter.SENSOR_TYPE_AZURE_KINECT_DK, SENSOR_NO_MERGE)

            # debug
            skeletons_98 = []
            for cam_idx in range(len(moox_data_list)):
                for human_idx in range(len(moox_data_list[cam_idx]["skeletons"])):
                    # カメラindexもjsonに追記
                    moox_data_list[cam_idx]["skeletons"][human_idx]['camera_idx'] = cam_idx

                skeletons_98.extend(moox_data_list[cam_idx]["skeletons"])

            data_98 = {
                "skeletons" : skeletons_98
            }
            save_upload_data(data_98, predict_json_getter.SENSOR_TYPE_AZURE_KINECT_DK, SENSOR_NO_MOOX)

            time.sleep(0.08)
        except:
            traceback.print_exc()

    print("処理完了")

def get_dif_moox_coordinate(camera_data_list):
    """
    各カメラ座標とmoox座標との差を算出する。
    ※カメラデータの人物は一人のみであること

    Args:
        camera_data_list(list(camera_data)):同Timestamp(とみなせる)各カメラのnuitrack jsonデータからなるリスト

    Returns:
        ret_list:平行位置差、回転行列のリスト
    """
    ORIGIN_JOINT_NUM = K4ABT_JOINT_HEAD
    Y_DIRECTION_JOINT_NUM = K4ABT_JOINT_NECK
    ORIGIN_JOINT_NUM2 = K4ABT_JOINT_SHOULDER_LEFT
    Y_DIRECTION_JOINT_NUM2 = K4ABT_JOINT_ELBOW_LEFT
    THRESHOLD = 0.75

    ret_list = []
    for camera_data in camera_data_list:
        #バリデーションチェック
        if len(camera_data["skeletons"]) == 0:
            print("skeletonsが無いデータが存在します")
            return None

        skeleton = camera_data["skeletons"][0]

        origin_joint_num = None
        y_direction_joint_num = None
        origin_joint_num2 = None
        y_direction_joint_num2 = None

        origin_joint_num = ORIGIN_JOINT_NUM
        y_direction_joint_num = Y_DIRECTION_JOINT_NUM
        origin_joint_num2 = ORIGIN_JOINT_NUM2
        y_direction_joint_num2 = Y_DIRECTION_JOINT_NUM2

        moox_origin_real = np.array(skeleton["joints"][origin_joint_num]["position"])
        moox_y_real = np.array(skeleton["joints"][y_direction_joint_num]["position"])
        moox_y_vector = moox_y_real - moox_origin_real

        moox_origin_real2 = np.array(skeleton["joints"][origin_joint_num2]["position"])
        moox_y_real2 = np.array(skeleton["joints"][y_direction_joint_num2]["position"])
        moox_y_vector2 = moox_y_real2 - moox_origin_real2

        print("----start----")

        #moox_y_vector = np.array([1.0, 1.0, 1.0])

        print("---->")
        v0 = copy.deepcopy(moox_y_vector)
        print("v0 {}".format(v0))

        corner_x = 0
        corner_y = 0
        corner_z = 0

        print("---->")
        corner_x = getAngle([v0[1], v0[2]], [1.0, 0.0])
        v1 = np.dot(getRotationMatrix(corner_x, corner_y, corner_z), v0)

        if abs(v1[2]) > 0.01:
            print("  if abs(v0[2]) > 0.01:")
            corner_x *= -1
            v1 = np.dot(getRotationMatrix(corner_x, corner_y, corner_z), v0)

        print("v1 {}".format(v1))

        print("---->")
        corner_z = getAngle([v1[0], v1[1]], [0.0, 1.0])
        v2 = np.dot(getRotationMatrix(corner_x, corner_y, corner_z), v0)

        if abs(v2[0]) > 0.01:
            print(" if abs(v0[0]) > 0.01:")
            corner_z *= -1
            v2 = np.dot(getRotationMatrix(corner_x, corner_y, corner_z), v0)

        print("v2 {}".format(v2))

        print("---->")
        corner_y = getAngle([moox_y_vector2[0], moox_y_vector2[2]], [1.0, 0.0])
        v3 = np.dot(getRotationMatrix(corner_x, corner_y, corner_z), v0)
        if abs(v3[0]) > 0.01:
            print(" abs(v3[0]) > 0.01:")
            corner_y *= -1

        corner_y -= np.deg2rad(90)
        print("v3 {}".format(v3))

        print("---->")
        print([np.rad2deg(corner_x), np.rad2deg(corner_y), np.rad2deg(corner_z)])

        calc_pos = np.dot(getRotationMatrix(corner_x, corner_y, corner_z), v0)
        print("v0 {}".format(v0))
        print("calc_pos {}".format(calc_pos))

        print("---end")
        print("")

        dif_real = np.zeros(3) - moox_origin_real
        dif_orient = getRotationMatrix(corner_x, corner_y, corner_z)

        # orient使用版
        # origin_joint_orient = skeleton["joints"][ORIGIN_JOINT_NUM]["orient"]
        # dif_orient = np.array(origin_joint_orient).reshape(3, 3)
        # dif_orient = np.linalg.inv(dif_orient)

        ret_list.append({
            'position' : {
                'x' : dif_real[0],
                'y' : dif_real[1],
                'z' : dif_real[2],
            },
            'orient' : {
                'x' : round(np.rad2deg(corner_x), 3),
                'y' : round(np.rad2deg(corner_y), 3),
                'z' : round(np.rad2deg(corner_z), 3),
            }
        })

    return ret_list

def getAngle(_v1, _v2):
    #print("{} - {}".format(_v1, _v2))
    v1 = np.array(_v1)
    v2 = np.array(_v2)

    if np.linalg.norm(v1) == 0 or np.linalg.norm(v2) == 0:
        return 0

    i = np.dot(v1, v2)
    n = np.linalg.norm(v1) * np.linalg.norm(v2)

    c = i / n

    r = np.arccos(np.clip(c, -1.0, 1.0))
    a = np.rad2deg(r)

    # print("angle {}".format(a))

    return r

# def getAngle_(_v1, _v2):
#     v1 = np.array(_v1)
#     v2 = np.array(_v2)

#     if np.linalg.norm(v1) == 0 or np.linalg.norm(v2) == 0:
#         return 0

#     i = _v1[0] * _v2[0] + _v1[1] * _v2[1] + _v1[1] * _v2[1]

#     n = np.sqrt(np.power(_v1[0], 2) + np.power(_v1[1], 2) + np.power(_v1[2], 2)) \
#         * np.sqrt(np.power(_v2[0], 2) + np.power(_v2[1], 2) + np.power(_v2[2], 2))

#     c = i / n

#     r = np.arccos(np.clip(c, -1.0, 1.0))
#     a = np.rad2deg(r)

#     return r

def getRotationMatrix(corner_x, corner_y, corner_z):
    Rx = np.array([1, 0, 0, 0, math.cos(corner_x), -math.sin(corner_x), 0, math.sin(corner_x), math.cos(corner_x)]).reshape(3, 3)
    Ry = np.array([math.cos(corner_y), 0, math.sin(corner_y), 0, 1, 0, -math.sin(corner_y), 0, math.cos(corner_y)]).reshape(3, 3)
    Rz = np.array([math.cos(corner_z), -math.sin(corner_z), 0, math.sin(corner_z), math.cos(corner_z), 0, 0, 0, 1]).reshape(3, 3)
    R = Rx.dot(Rz).dot(Ry)

    R = np.linalg.inv(R)

    return R

def getRotationMatrix2(d_x, d_y, d_z):

    print("{} {} {}".format((d_x % 360), (d_y % 360), (d_z % 360)))

    corner_x = np.deg2rad(d_x)
    corner_y = np.deg2rad(d_y)
    corner_z = np.deg2rad(d_z)

    Rx = np.array([1, 0, 0, 0, math.cos(corner_x), -math.sin(corner_x), 0, math.sin(corner_x), math.cos(corner_x)]).reshape(3, 3)
    Ry = np.array([math.cos(corner_y), 0, math.sin(corner_y), 0, 1, 0, -math.sin(corner_y), 0, math.cos(corner_y)]).reshape(3, 3)
    Rz = np.array([math.cos(corner_z), -math.sin(corner_z), 0, math.sin(corner_z), math.cos(corner_z), 0, 0, 0, 1]).reshape(3, 3)
    R = Rz.dot(Ry).dot(Rx)

    R = np.linalg.inv(R)

    return R

def getRx(corner_x):
    return np.array([1, 0, 0, 0, math.cos(corner_x), -math.sin(corner_x), 0, math.sin(corner_x), math.cos(corner_x)]).reshape(3, 3)

def getRy(corner_y):
    return np.array([math.cos(corner_y), 0, math.sin(corner_y), 0, 1, 0, -math.sin(corner_y), 0, math.cos(corner_y)]).reshape(3, 3)

def getRz(corner_z):
    return np.array([math.cos(corner_z), -math.sin(corner_z), 0, math.sin(corner_z), math.cos(corner_z), 0, 0, 0, 1]).reshape(3, 3)

def get_moox_data(camera_data, calibration_data, checkerboard_data):
    """
    カメラ起点座標系からmoox座標系に変換したデータを取得する

    Args:
        camera_datas(list(camera_data)):同一カメラのnuitrack jsonデータからなるリスト
        calibration_data(tuple(real,orient):各カメラのmoox座標系との位置差データ

    Returns:
        ret_datas:moox座標系変換後のカメラデータ
    """

    ret_data = copy.deepcopy(camera_data)

    #バリデーションチェック
    if len(ret_data["skeletons"]) == 0:
        # print("skeletonsが無いデータが存在します")
        return ret_data

    for skeleton in ret_data["skeletons"]:
        for joint in skeleton["joints"]:
            print(joint["position"])
            cam_pos = np.array(joint["position"]).reshape(3,1)

            # 角度計算
            if merge_mode == 1:
                rotation_matrix = getRotationMatrix(
                    np.deg2rad(calibration_data['orient']['x']),
                    np.deg2rad(calibration_data['orient']['y']),
                    np.deg2rad(calibration_data['orient']['z'])
                ).reshape(3, 3)

            elif merge_mode == 2:
                rotation_matrix = np.array(checkerboard_data["r_matrix"]).reshape(3, 3)
                # print(rotation_matrix)

            elif merge_mode == 3:
                rotation_matrix = np.array(checkerboard_data["r_matrix"]).reshape(3, 3)

            # ズレ計算
            if merge_mode == 1:
                moox_pos_diff = np.array([calibration_data['position']['x'], calibration_data['position']['y'], calibration_data['position']['z']]).reshape(3,1)

            elif merge_mode == 2:
                x_scalar = checkerboard_data["t_vecs"][0][0][0]
                y_scalar = checkerboard_data["t_vecs"][0][1][0]
                z_scalar = checkerboard_data["t_vecs"][0][2][0]
                # print(checkerboard_data["t_vecs"])

                z_vector = np.array([x_scalar, y_scalar, z_scalar]).astype(np.float32).reshape(3,1)
                moox_pos_diff = -np.dot(rotation_matrix, z_vector)

            elif merge_mode == 3:
                moox_pos_diff = np.array([calibration_data['position']['x'], calibration_data['position']['y'], calibration_data['position']['z']]).reshape(3,1)


            # 原点へ移動
            moox_pos = cam_pos + moox_pos_diff

            #　回転
            #real_moox = np.dot(rotation_matrix, moox_pos)
            print(joint["position"])
            real_moox = np.dot(rotation_matrix, cam_pos) + z_vector

            joint["position"] = [
                real_moox[0][0],
                real_moox[1][0],
                real_moox[2][0]
            ]
            print(joint["position"])

            # 方向回転
            # joint["orientation"]
            # orient_moox = rotation_matrix * np.array(joint["orient"]).reshape(3, 3)
            # joint["orient"] = orient_moox.reshape(1,9).tolist()

    return ret_data

def calibration_accurate(camera_data_list, moox_data_list, calibration_data_list):
    TARGET_JOINT_NUM = K4ABT_JOINT_HEAD  #JOINT_HEAD

    real_list = []
    # for camera_idx in range(len(moox_data_list)):
    #     # 各カメラの関節座標を取得
    #     if len(moox_data_list[camera_idx]["skeletons"]) == 0:
    #         print("accurate skeletonsが無いデータが存在します")
    #         break

    #     s_idx = 0
    #     real = np.array([moox_data_list[camera_idx]["skeletons"][s_idx]["joints"][TARGET_JOINT_NUM]["real"][axis] for axis in ("x", "y", "z")])

    #     real_list.append(real)
    real_list=[np.array([39.2894,298.678,1316.86]),np.array([-138.489,430.011,1083.2])]

    if len(real_list) == len(moox_data_list):

        for ri, _real in enumerate(real_list):
            v0 = copy.deepcopy(_real).reshape(3)
            vy = np.array([0,1,0]).reshape(3)

            print("vector{} : {}".format(ri, v0))
            corner_x = 0
            corner_y = 0
            corner_z = 0

            # yz平面上でy軸に合わせる
            corner_x = getAngle([v0[1], v0[2]], [vy[1], vy[2]])
            v1 = np.dot(getRx(corner_x), v0).reshape(3)
            # v1のz要素が0とみなせる値になっていなければ回転方向を逆に修正
            if abs(v1[2]) > 0.01:
                corner_x *= -1
                v1 = np.dot(getRx(corner_x), v0).reshape(3)

            # xy平面上でy軸に合わせる
            corner_z = getAngle([v1[0], v1[1]], [vy[0], vy[1]])
            v2 = np.dot(getRz(corner_z), v1).reshape(3)
            # v2のz要素が0とみなせる値になっていなければ回転方向を逆に修正
            if abs(v2[0]) > 0.01:
                corner_z *= -1
                v2 = np.dot(getRz(corner_z), v1).reshape(3)

            # y軸方向への修正はこのベクトルからは算出できない
            corner_x = round(np.rad2deg(corner_x), 3)
            corner_z = round(np.rad2deg(corner_z), 3)
            print("-->rotaion diff{} : x:{}, z:{}".format(ri, corner_x, corner_z))

            if (ri == 0):
                base_corners = [corner_x, corner_z]
            else:
                dif_corner_x = base_corners[0] - corner_x
                dif_corner_z = base_corners[1] - corner_z

                v1_= np.dot(getRx(dif_corner_x), v0).reshape(3)
                v2_ = np.dot(getRz(dif_corner_z), v1).reshape(3)

                # 変換テスト(before)
                ret = get_moox_data(camera_data_list[ri], calibration_data_list[ri])

                # # キャリブレーションデータを補正
                calibration_data_list[ri]['orient']['x'] -= dif_corner_x
                calibration_data_list[ri]['orient']['y'] -= dif_corner_y
                calibration_data_list[ri]['orient']['z'] -= dif_corner_z

                # 変換テスト(after)
                ret = get_moox_data(camera_data_list[ri], calibration_data_list[ri])

                # キャリブレーション値保存
                save_upload_data(calibration_data_list, predict_json_getter.SENSOR_TYPE_AZURE_KINECT_DK_CALIBRATION, SENSOR_NO_MERGE)

    else:
        print("accurate データ不足")
        pass

def get_merged_data(moox_data_list, pre_merged_data):
    """
    複数のmoox座標データからマージ値を取得する
    """

    COMPARE_JOINT_NUM = K4ABT_JOINT_HEAD
    SAME_HUMAN_CAMERA_DISTANCE_THRESHOLD = 700
    SAME_HUMAN_MOVE_DISTANCE_THRESHOLD = 300

    # カメラごとでの同一人物リスト作成
    human_list_list = []
    for camera_idx in range(len(moox_data_list)):
        for skeleton in moox_data_list[camera_idx]["skeletons"]:
            real = np.array(skeleton["joints"][COMPARE_JOINT_NUM]["position"])

            found = False
            found_hlli = -1
            min_distance = SAME_HUMAN_CAMERA_DISTANCE_THRESHOLD
            for hlli in range(len(human_list_list)):
                human_list = human_list_list[hlli]
                human = human_list[0] # 比較は１つ目の要素と行う

                #同一カメラで同一人物判定はしない
                if human["camera_idx"] == camera_idx:
                    continue

                # 距離計算
                real_compare = np.array(human["skeleton"]["joints"][COMPARE_JOINT_NUM]["position"])
                distance = np.linalg.norm(real - real_compare)

                # 一定以下かどうか
                if distance < min_distance:
                    min_distance = distance
                    found = True
                    found_hlli = hlli

            human = {
                "skeleton" : skeleton,
                "camera_idx": camera_idx
            }

            if found:
                human_list = human_list_list[found_hlli]
                human_list.append(human)
            else:
                human_list = [human]
                human_list_list.append(human_list)

    # 同一人物座標マージ処理
    merge_data = {
            "skeletons": []
        }
    USE_AVERAGE = False
    if USE_AVERAGE:
        for hlli in range(len(human_list_list)):
            human_list = human_list_list[hlli]

            joints = []
            for ji in range(len(human_list[0]["skeleton"]["joints"])):
                to_merge_list =[]
                for hli in range(len(human_list)):
                    human = human_list[hli]
                    confidence = human["skeleton"]["joints"][ji]["confidence"]
                    real = np.array(human["skeleton"]["joints"][ji]["position"])
                    orient = np.array(human["skeleton"]["joints"][ji]["orientation"])
                    to_merge_list.append({
                        "confidence": confidence,
                        "position": real,
                        "orientation": orient
                    })

                max_confidence = 0
                sum_confidence = 0
                sum_real_conf = 0
                sum_orient_conf = 0
                for data in to_merge_list:
                    max_confidence = max(max_confidence, data["confidence"])
                    sum_confidence += data["confidence"]
                    sum_real_conf += data["position"] * data["confidence"]
                    sum_orient_conf += data["orientation"] * data["confidence"]

                #マージ値算出
                merged_real = None
                merged_orient = None
                if sum_confidence == 0:
                    merged_real = np.zeros(3)
                    merged_orient = np.zeros(9)
                else:
                    merged_real = sum_real_conf / sum_confidence
                    merged_orient = sum_orient_conf / sum_confidence

                #反映
                joints.append({
                    "position" : merged_real,
                    "confidence": max_confidence,
                    "type": ji,
                    "proj": {
                        "z": 0,
                        "x": 0,
                        "y": 0
                    },
                    "orientation" : merged_orient.tolist()
                })

            merge_data["skeletons"].append({
                "joints" : joints,
                "id": -1
            })
    else:
        for hlli in range(len(human_list_list)):
            human_list = human_list_list[hlli]

            raw_camera_ids = {}
            for hli in range(len(human_list)):
                camera_idx = human_list[hli]["camera_idx"]
                human_id = human_list[hli]["skeleton"]["id"]
                raw_camera_ids[camera_idx] = human_id

            if len(human_list) >= 2:
                sum_conf_list = []

                for hli in range(len(human_list)):
                    sum_conf = 0
                    for ji in range(len(human_list[hli]["skeleton"]["joints"])):
                        confidence = 1 # human["skeleton"]["joints"][ji]["confidence"]
                        sum_conf += confidence
                    sum_conf_list.append(sum_conf)

                use_human_index = sum_conf_list.index(max(sum_conf_list))
            else:
                use_human_index = 0

            merge_data["skeletons"].append({
                "joints" : human_list[use_human_index]["skeleton"]["joints"],
                "id" : -1,
                "raw_camera_ids" : raw_camera_ids
            })

    # compare_jointのmoox_dataとmerge_dataの距離差を追記
    for cam_idx in range(len(moox_data_list)):
        for human_idx in range(len(moox_data_list[cam_idx]["skeletons"])):

            real = np.array(moox_data_list[cam_idx]["skeletons"][human_idx]["joints"][COMPARE_JOINT_NUM]["position"])

            min_diff = None
            for merge_idx in range(len(merge_data["skeletons"])):

                merge_real = np.array(merge_data["skeletons"][merge_idx]["joints"][COMPARE_JOINT_NUM]["position"])

                dif_real = np.linalg.norm(real - merge_real)

                if min_diff is None or min_diff > dif_real:
                    min_diff = dif_real

            if min_diff is None:
                min_diff = 0

            moox_data_list[cam_idx]["skeletons"][human_idx]['distance_from_merge_data'] = min_diff

    # 人物ID割当 過去フレームとのマージ
    used_id_list = []
    for si in range(len(merge_data["skeletons"])):
        real = np.array(merge_data["skeletons"][si]["joints"][COMPARE_JOINT_NUM]["position"])

        found = False
        found_id = -1
        min_distance = SAME_HUMAN_MOVE_DISTANCE_THRESHOLD

        if pre_merged_data is not None:
            for psi in range(len(pre_merged_data["skeletons"])):

                # 距離計算
                real_compare = np.array(pre_merged_data["skeletons"][psi]["joints"][COMPARE_JOINT_NUM]["position"])
                distance = np.linalg.norm(real - real_compare)

                # 一定以下かどうか
                if distance < min_distance:
                    min_distance = distance
                    found = True
                    found_id = pre_merged_data["skeletons"][psi]["id"]

        if found:
            merge_data["skeletons"][si]["id"] = found_id
            used_id_list.append(found_id)

    # 人物ID割当　新キャラ
    for si in range(len(merge_data["skeletons"])):
        if merge_data["skeletons"][si]["id"] >= 0:
            continue

        for i in range(1000000):
            if i in used_id_list:
                continue

            merge_data["skeletons"][si]["id"] = i
            used_id_list.append(i)
            break

    exclude_impposible_joints(merge_data)

    return merge_data

def exclude_impposible_joints(merge_data):
    '''
    ありえない関節位置の排除
    '''
    pass


#sqlダンプファイルからデータ読み込み
def read_sql_file(path):
    camera1_data = []
    camera2_data = []
    a = os.path.abspath(path)
    with open(path, 'r', encoding='utf-8') as f:
        for row in csv.reader(f):
            if any(field.strip() for field in row) and row[0][0] == '(':
                nuitrack = ",".join(row[5:])
                nuitrack = nuitrack.replace("\\","")
                nuitrack=nuitrack.replace("'", "")
                nuitrack=re.sub(r"\),$", "", nuitrack)
                nuitrack=re.sub(r"\);$", "", nuitrack)

                nui_json=json.loads(nuitrack)

                if len(nui_json["skeletons"]) > 0:
                    if int(row[2]) == 1:
                        camera1_data.append(nui_json)
                    if int(row[2]) == 2:
                        camera2_data.append(nui_json)

    return (camera1_data, camera2_data)


#ファイル書き出し
def write_file(path, list):
    with open(path, 'w', encoding='utf-8') as f:
        for x in list:
            f.write(str(x) + "\n")

def call_moox_api(name, params):
    host = moox_api_address
    p = urllib.parse.urlencode(params)
    url = "http://{}/{}?{}".format(host, name, p)
    print(url)

    with urllib.request.urlopen(url) as res:
        html = res.read().decode("utf-8")
        print(html)


def get_area_person(merged_data, compare_joint_num, point, threshold_distance):
    print("get_area_person")

    for psi in range(len(merged_data["skeletons"])):

        # 距離計算
        compare_pos = np.array(merged_data["skeletons"][psi]["joints"][compare_joint_num]["position"])
        distance = np.linalg.norm(point - compare_pos)


        print(compare_pos)
        print(distance)

        # 一定以下かどうか
        if distance < threshold_distance:
            return merged_data["skeletons"][psi]

    return None

def check_need_capture(merged_data):

    # COMPARE_JOINT_NUM = K4ABT_JOINT_HEAD
    # CAPTURE_POINT = np.array([0 ,0, 0])
    # THRESHOLD_DISTANCE = 300

    # person = get_area_person(merged_data, COMPARE_JOINT_NUM, CAPTURE_POINT, THRESHOLD_DISTANCE)

    # if person is None:
    #     return False
    # else:
    #     return True

    for psi in range(len(merged_data["skeletons"])):

        y_idx = 2

        # 距離計算
        head_pos = np.array(merged_data["skeletons"][psi]["joints"][K4ABT_JOINT_HEAD]["position"])
        l_wrist_pos = np.array(merged_data["skeletons"][psi]["joints"][K4ABT_JOINT_WRIST_LEFT]["position"])
        r_wrist_pos = np.array(merged_data["skeletons"][psi]["joints"][K4ABT_JOINT_WRIST_RIGHT]["position"])

        hip_pos = np.array(merged_data["skeletons"][psi]["joints"][K4ABT_JOINT_HIP_RIGHT]["position"])
        r_knee_pos = np.array(merged_data["skeletons"][psi]["joints"][K4ABT_JOINT_KNEE_RIGHT]["position"])
        r_ankle_pos = np.array(merged_data["skeletons"][psi]["joints"][K4ABT_JOINT_ANKLE_RIGHT]["position"])

        # 手を上げている
        if l_wrist_pos[y_idx] < head_pos[y_idx] and r_wrist_pos[y_idx] < head_pos[y_idx]:
            continue

        # 立っている
        y1 = abs(hip_pos[y_idx] - r_knee_pos[y_idx])
        y2 = abs(r_knee_pos[y_idx] - r_ankle_pos[y_idx]) * 0.5

        if y1 < y2:
            continue

        print("★cap {}".format([head_pos, l_wrist_pos, r_wrist_pos]))
        print("★cap2 {}".format([hip_pos, r_knee_pos, r_ankle_pos]))
        print("★cap3 {}".format([y1, y2]))
        return True

    return False

def check_person_position(merged_data):

    COMPARE_JOINT_NUM = K4ABT_JOINT_HEAD
    CAPTURE_POINT = np.array([0 ,0, 0])
    THRESHOLD_DISTANCE = 600

    SEAT_1_POINT = np.array([900,  430,  300])
    SEAT_2_POINT = np.array([-750, 430,  300])
    SEAT_3_POINT = np.array([0,    430, -600])

    sitting_data = {
    }

    seat1 = get_area_person(merged_data, COMPARE_JOINT_NUM, SEAT_1_POINT, THRESHOLD_DISTANCE)
    seat2 = get_area_person(merged_data, COMPARE_JOINT_NUM, SEAT_2_POINT, THRESHOLD_DISTANCE)
    seat3 = get_area_person(merged_data, COMPARE_JOINT_NUM, SEAT_3_POINT, THRESHOLD_DISTANCE)

    sitting_data["seat1"] = seat1
    sitting_data["seat2"] = seat2
    sitting_data["seat3"] = seat3

    save_upload_data(sitting_data, predict_json_getter.SENSOR_TYPE_PERSON_POSITION, 1)


if __name__ == "__main__":
    main()
    # test_calibration_accurate()
