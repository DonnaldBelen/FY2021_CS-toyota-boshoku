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
import cv2
import glob
import pprint
import shutil

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../predict/')
import predict_json_getter
from datetime import date, datetime, timedelta

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../upload/')
from upload_data import upload_json    

SENSOR_TYPE_NUITRACK_CHECKERBOARD_CALIBRATION = '73'

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

# arg
parser = argparse.ArgumentParser(description='nuitrack')
parser.add_argument('--preview', type=int, default=0) # プレビュー
parser.add_argument('--infinity', type=int, default=0)
parser.add_argument('--camera_no', type=int, default=0) # プレビュー
args = parser.parse_args()

sensor_no = args.camera_no

# ソートされた連番の画像ファイルパスを取得できます
def numerical_sort(value):
    numbers = re.compile(r'(\d+)')
    parts = numbers.split(value)
    parts[1::2] = map(int, parts[1::2])
    return parts

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
    print(frame_data)

def main():
    print("処理開始")

    try:
        # ret = False
        # idx = -1 
        # while not ret:
        #     cap = cv2.VideoCapture(idx)
        #     ret, frame = cap.read()
        #     idx += 1
        #     if ret or idx > 20:
        #         break
        # if not ret:
        #     print("video capture failed")
        #     return

        #images = [frame]

        image_path_list = []
        for path in sorted(glob.glob('/ramdisk/capture_*.png'), key=numerical_sort):
            image_path_list.append(path)

        ret, t_vecs, r_matrix = checker_boad_calibrate(image_path_list)

        for path in image_path_list:
            os.remove(path)

        if not ret:
            return

        save_upload_data({
            't_vecs' : t_vecs,
            'r_matrix' : r_matrix
        }, SENSOR_TYPE_NUITRACK_CHECKERBOARD_CALIBRATION, sensor_no)

        upload_json(False)

        print("完了")
    except:
        traceback.print_exc()

def checker_boad_calibrate(image_path_list):
    global args

    calibration_board_type = 1 # 0:checker 1:circle

    if calibration_board_type == 0:
        BOARD_ROWS = 10 # 検出するチェッカーボードの格子点行数
        BOARD_COLS = 7 # 検出するチェッカーボードの格子点列数
        BOARD_SCALE = 74 # チェッカーボードの格子一辺のスケール(mm)
        # BOARD_ROWS = 8 # 検出するチェッカーボードの格子点行数
        # BOARD_COLS = 6 # 検出するチェッカーボードの格子点列数
        # BOARD_SCALE = 40 # チェッカーボードの格子一辺のスケール(mm)
    elif calibration_board_type == 1:
        BOARD_ROWS = 4
        BOARD_COLS = 5
        BOARD_SCALE = 154
    else:
        return False, None, None

    if calibration_board_type == 0:
        # チェッカーボードの格子点の実世界三次元座標を作成する
        # ボードは平面なのでz軸を０として (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)　のようにする（適宜スケールをかける）
        objpoint = np.zeros((BOARD_COLS*BOARD_ROWS,3), np.float32)
        objpoint[:,:2] = np.mgrid[0:BOARD_ROWS,0:BOARD_COLS].T.reshape(-1,2) * BOARD_SCALE

    elif calibration_board_type == 1:
        objpoint = np.zeros((BOARD_COLS*BOARD_ROWS,3), np.float32)

        idx = 0    
        for ix in reversed(range(BOARD_COLS)):
            for iy in range(BOARD_ROWS):
                x = BOARD_SCALE * ix
                y = BOARD_SCALE * iy

                objpoint[idx] = np.array([x, y, 0])
                idx += 1

        objpoint = np.array(objpoint)

    # CALIB_CB_ASYMMETRIC_GRID 用
    # # 抽出はXmax,Ymax -> Xmin,Ymax -> Ymin,Xminの順
    # # ボードは平面なのでz軸を０として (10,10,0), (8,10,0), (6,10,0) ...., (9,9,0), (7,9,0) ....,(0,0,0)　のようにする（適宜スケールをかける）
    # for iy in reversed(range(BOARD_COLS)):
    #     for ix in reversed(range(BOARD_ROWS)):
    #         px = (0.5 if (iy % 2 == 0) else 0)
    #         x = BOARD_SCALE * (ix + px)
    #         y = BOARD_SCALE / 2.0 * iy

    #         objpoint.append(np.array([x, y, 0]))

    # objpoint = np.array(objpoint)

    objpoints = [] 
    imgpoints = [] 

    # ボード画像ループ
    for i, path in enumerate(image_path_list):
        print("check {}".format(path))

        original_img = cv2.imread(path)

        # リサイズ
        orgHeight, orgWidth = original_img.shape[:2]
        scale = orgWidth / 1000.0

        if scale > 1: 
            size = (int(orgWidth / scale), int(orgHeight / scale))
            original_img = cv2.resize(original_img, size)

        # グレースケール化
        gray = cv2.cvtColor(original_img,cv2.COLOR_BGR2GRAY)

        # カメラ座標上のチェッカーボードの格子点座標を取得
        if calibration_board_type == 0:
            ret, corners_rough = cv2.findChessboardCorners(gray, (BOARD_ROWS,BOARD_COLS),None)
        elif calibration_board_type == 1:
            ret, corners_accurate = cv2.findCirclesGrid(gray, (BOARD_ROWS,BOARD_COLS), flags=cv2.CALIB_CB_SYMMETRIC_GRID + cv2.CALIB_CB_CLUSTERING)
            
        if ret:            
            if calibration_board_type == 0:
                # 格子点座標精度上げ
                win_size = (11, 11) # 探索窓のサイズの半分．例えば winSize=Size(5,5) の場合，5*2+1  \times 5*2+1 = 11  \times 11 サイズの探索窓が利用されます(原文ママ：http://opencv.jp/opencv-2.1/cpp/feature_detection.html)
                zero_zone = (-1, -1) # 探索領域の中心に存在する対象外領域（後述の式において総和を計算する際に含まれない）の半分のサイズ．この値は，自己相関行列において発生しうる特異点を避けるために用いられます． 値が (-1,-1) の場合は，そのようなサイズはないということを意味します(原文ママ：http://opencv.jp/opencv-2.1/cpp/feature_detection.html)
                criteria_type = cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER # 最大繰り返し数または収束閾値未満になったら終了する
                MAX_ITERATION_COUNT = 30  # 最大繰り返し数
                epsiron = 0.001 # 収束閾値

                criteria = (criteria_type, MAX_ITERATION_COUNT, epsiron)
                corners_accurate = cv2.cornerSubPix(gray, corners_rough, win_size, zero_zone, criteria)
            
            elif calibration_board_type == 1:
                first = corners_accurate[0][0]
                last = corners_accurate[-1][0]

                if first[0] < last[0]:
                    corners_accurate = corners_accurate[::-1]

            objpoints.append(objpoint)
            imgpoints.append(corners_accurate)

            print("objpoint")
            print(objpoint[:20])
            print("corners_accurate")
            print(corners_accurate[:20])

            # 元画像に格子点を描画する
            img_accurate = cv2.drawChessboardCorners(original_img.copy(), (BOARD_ROWS, BOARD_COLS), corners_accurate, ret)

            cv2.imshow('preview', img_accurate)
        else:
            cv2.imshow('preview', original_img)

        # 無限ループ時は１枚のみ
        if args.infinity == 1:
            break;
            
    if args.preview == 1:
        cv2.waitKey(1)

    if len(objpoints) == 0 and len(imgpoints) == 0:
        print("チェッカーボードの格子点を必要数見つけられませんでした。")
        return False, None, None

    # カメラキャリブレーション値を算出 
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    Rs = []
    for rvec in rvecs:
        # 回転ベクトルを回転行列に変換

        # if calibration_board_type == 1:
        #     rvec[0][0] = -rvec[0][0]

        rvec[1][0] = -rvec[1][0]

        R, jacob = cv2.Rodrigues(rvec)
        Rs.append(R)

    print("ret")
    pprint.pprint(ret)
    print("----------")
    print("cameraMatrix")
    pprint.pprint(mtx)
    print("----------")
    print("distCoeffs")
    pprint.pprint(dist)
    print("----------")
    print("回転ベクトル")
    pprint.pprint(rvecs)
    print("回転行列")
    pprint.pprint(Rs)
    print("----------")
    print("平行移動ベクトル")
    pprint.pprint(tvecs)

    return True ,tvecs, Rs

if __name__ == "__main__":
    if sys.argv:
        del sys.argv[1:]

    main()
    while args.infinity == 1:
        time.sleep(0.1)
        main()
    
    cv2.destroyAllWindows()
