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
from cs_common.common import MyEncoder
from cs_common.common import CSNode
import cs_common.common
from rclpy.executors import MultiThreadedExecutor
import mysql.connector
import mysql.connector.pooling
import subprocess
import json
from datetime import date, datetime, timedelta
import re
import copy
import numpy as np
import argparse

class CSAzureKinectMerge(CSNode):
    def __init__(self):
        super().__init__('CSAzureKinectMerge')

        parser = argparse.ArgumentParser(description='CSAzureKinectMerge')
        parser.add_argument('--num', type=int, default=0) # NO
        args = parser.parse_args()

        self.num = args.num

        self.last_merge_time = time.time()
        self.timer = self.create_timer(0.01, self.merge)

        self.camera_data_list = [None] * self.num
        self.checkerboard_data_list = [None] * self.num

        azure_kinect_calibration_dir = os.path.dirname(os.path.abspath(__file__))
        azure_kinect_calibration_path = "%s/azure_kinect_calibration_0.json" % azure_kinect_calibration_dir
        for i in range(10):
            if os.path.exists(azure_kinect_calibration_path):
                break
            else:
                azure_kinect_calibration_dir = "%s/.." % azure_kinect_calibration_dir
                azure_kinect_calibration_path = "%s/azure_kinect_calibration_0.json" % azure_kinect_calibration_dir
        self.get_logger().info("load %s" % azure_kinect_calibration_path)

        for i in range(self.num):
            azure_kinect_calibration_path = "%s/azure_kinect_calibration_%d.json" % (azure_kinect_calibration_dir, i)
            with open(azure_kinect_calibration_path) as f:
                tmp = json.load(f)
                self.checkerboard_data_list[i] = tmp['data']

        self.data_cnt = 0
        self.pre_data_cnt = -1

        self.pre_merged_data = None

    def run(self):
        qos_profile = CSQoS.create(CSQoS.QoSType.SENSOR)

        subnode1 = CSSubscriberNode('/cs/data/sensor/azure_kinect/c_0',
            String, qos_profile, self.callback_subscribe0)

        subnode2 = CSSubscriberNode('/cs/data/sensor/azure_kinect/c_1',
            String, qos_profile, self.callback_subscribe1)

        executor = MultiThreadedExecutor(num_threads=10)
        executor.add_node(subnode1)
        executor.add_node(subnode2)
        executor.add_node(self)

        try:
            executor.spin()
        except KeyboardInterrupt:
            self.get_logger().warning("KeyboardInterrupt!")

        executor.shutdown()

        subnode1.destroy_node()
        subnode2.destroy_node()

    def callback_subscribe0(self, msg):
        tmp = json.loads(msg.data)
        self.camera_data_list[0] = tmp

        self.data_cnt += 1

    def callback_subscribe1(self, msg):
        tmp = json.loads(msg.data)
        self.camera_data_list[1] = tmp
        self.data_cnt += 1

    def get_cs_data(self, camera_data, checkerboard_data):
        """
        カメラ起点座標系からcs座標系に変換したデータを取得する

        Args:
            camera_datas(list(camera_data)):同一カメラのnuitrack jsonデータからなるリスト

        Returns:
            ret_datas:cs座標系変換後のカメラデータ
        """

        if camera_data is None:
            return None

        ret_data = copy.deepcopy(camera_data)

        #バリデーションチェック
        if len(ret_data["skeletons"]) == 0:
            return ret_data

        for skeleton in ret_data["skeletons"]:
            for joint in skeleton["joints"]:
                cam_pos = np.array(joint["position"]).reshape(3,1)

                # 角度計算
                rotation_matrix = np.array(checkerboard_data["r_matrix"]).reshape(3, 3)

                # ズレ計算
                x_scalar = checkerboard_data["t_vecs"][0][0][0]
                y_scalar = checkerboard_data["t_vecs"][0][1][0]
                z_scalar = checkerboard_data["t_vecs"][0][2][0]

                z_vector = np.array([x_scalar, y_scalar, z_scalar]).astype(np.float32).reshape(3,1)
                cs_pos_diff = -np.dot(rotation_matrix, z_vector)

                # 原点へ移動
                cs_pos = cam_pos + cs_pos_diff

                #　回転
                #real_cs = np.dot(rotation_matrix, cs_pos)
                real_cs = np.dot(rotation_matrix, cam_pos) + z_vector

                joint["position"] = [
                    real_cs[0][0],
                    real_cs[1][0],
                    real_cs[2][0]
                ]

                # 方向回転
                # joint["orientation"]
                # orient_cs = rotation_matrix * np.array(joint["orient"]).reshape(3, 3)
                # joint["orient"] = orient_cs.reshape(1,9).tolist()

        return ret_data

    def get_merged_data(self, cs_data_list, pre_merged_data):
        """
        複数のcs座標データからマージ値を取得する
        """

        COMPARE_JOINT_NUM = cs_common.common.K4ABT_JOINT_SPINE_NAVAL
        SAME_HUMAN_CAMERA_DISTANCE_THRESHOLD = 700
        SAME_HUMAN_MOVE_DISTANCE_THRESHOLD = 300

        # カメラごとでの同一人物リスト作成
        human_list_list = []
        for camera_idx in range(len(cs_data_list)):
            for skeleton in cs_data_list[camera_idx]["skeletons"]:
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

        # compare_jointのcs_dataとmerge_dataの距離差を追記
        for cam_idx in range(len(cs_data_list)):
            for human_idx in range(len(cs_data_list[cam_idx]["skeletons"])):

                real = np.array(cs_data_list[cam_idx]["skeletons"][human_idx]["joints"][COMPARE_JOINT_NUM]["position"])

                min_diff = None
                for merge_idx in range(len(merge_data["skeletons"])):

                    merge_real = np.array(merge_data["skeletons"][merge_idx]["joints"][COMPARE_JOINT_NUM]["position"])

                    dif_real = np.linalg.norm(real - merge_real)

                    if min_diff is None or min_diff > dif_real:
                        min_diff = dif_real

                if min_diff is None:
                    min_diff = 0

                cs_data_list[cam_idx]["skeletons"][human_idx]['distance_from_merge_data'] = min_diff

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

        self.exclude_impposible_joints(merge_data)

        return merge_data

    def exclude_impposible_joints(self, merge_data):
        '''
        ありえない関節位置の排除
        '''
        pass

    def merge(self):
        #マージデータ表示

        try:

            if sum(x is None for x in self.camera_data_list) > 0:
                return

            # apiデータがアップデートされているか判定
            if self.pre_data_cnt == self.data_cnt:
                return
            self.pre_data_cnt = self.data_cnt

            #キャリブレーション
            if len(self.camera_data_list) != len(self.checkerboard_data_list):
                return

            # cs座標系に変換
            cs_data_list = []
            for i in range(len(self.camera_data_list)):
                i_checkerboard = None
                if self.checkerboard_data_list is not None and len(self.checkerboard_data_list) > i:
                    i_checkerboard = self.checkerboard_data_list[i]

                cs_datas = self.get_cs_data(self.camera_data_list[i], i_checkerboard)
                cs_data_list.append(cs_datas)

            # 変換した座標をマージ
            merged_data = self.get_merged_data(cs_data_list, self.pre_merged_data)
            self.pre_merged_data = copy.deepcopy(merged_data)

            sensing_time = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S.%f")

            self.publish_data('/cs/data/sensor/azure_kinect/c_99', merged_data, sensing_time)

            # debug
            skeletons_98 = []
            for cam_idx in range(len(cs_data_list)):
                for human_idx in range(len(cs_data_list[cam_idx]["skeletons"])):
                    # カメラindexもjsonに追記
                    cs_data_list[cam_idx]["skeletons"][human_idx]['camera_idx'] = cam_idx

                skeletons_98.extend(cs_data_list[cam_idx]["skeletons"])

            data_98 = {
                "skeletons" : skeletons_98
            }
            self.publish_data('/cs/data/sensor/azure_kinect/c_98', merged_data, sensing_time)

        except:
            self.get_logger().error("%s" % traceback.format_exc())

        merge_time = time.time()
        elapsed = merge_time - self.last_merge_time
        self.last_merge_time = merge_time

        self.get_logger().info("merge FPS:{:.2f} elapsed {}".format(1.0 / elapsed, elapsed))

def main():
    args = []
    rclpy.init(args=args)

    node = CSAzureKinectMerge()
    node.run()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
