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
from rclpy.executors import MultiThreadedExecutor
import mysql.connector
import mysql.connector.pooling
import subprocess
import json
from datetime import date, datetime, timedelta
import re
import argparse

# 前処理用
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Detect_rule.detect_rule import Detect_rule
from Preprocessing.Preprocessor import Preprocessor

class CSDetectGesture(CSNode):
    def __init__(self):
        super().__init__('CSDetectGesture')

        parser = argparse.ArgumentParser(description='CSDetectGesture')
        parser.add_argument('--no', type=str, default="") # NO
        args = parser.parse_args()

        self.no = args.no

        self.get_logger().info("CSDetectesture NO => {}".format(self.no))

        self.prep = Preprocessor()

        self.detect_action = Detect_rule()

    def run(self):
        qos_profile = CSQoS.create(CSQoS.QoSType.SENSOR)

        subnode1 = CSSubscriberNode('/cs/data/sensor/sitting_seat_{}'.format(self.no),
            String, qos_profile, self.callback_subscribe)

        executor = MultiThreadedExecutor(num_threads=10)
        executor.add_node(subnode1)
        executor.add_node(self)

        try:
            executor.spin()
        except KeyboardInterrupt:
            self.get_logger().warning("KeyboardInterrupt!")

        executor.shutdown()

        subnode1.destroy_node()

    def callback_subscribe(self, msg):
        try:
            json_data = json.loads(msg.data)

            tic = time.time()

            # 入力データ取得
            sensing_time = json_data['sensing_time']
            dic_data = json_data

            out_box = []

            input_dict = self.prep.Calculate(dic_data['seat'])
            self.detect_action.Calculate(input_dict, sensing_time)
            dict_result = self.detect_action.dict_result

            # 処理時間計測
            #time_info = dic_data["time_info"]

            #d_time = datetime.utcnow()
            #s_time = datetime.strptime(sensing_time, "%Y-%m-%d %H:%M:%S.%f")
            #c_time = datetime.strptime(time_info["camera_sensing_time"]["0"], "%Y-%m-%d %H:%M:%S.%f")

            #time_info["detect_action_time"] = d_time.strftime("%Y-%m-%d %H:%M:%S.%f")
            #time_info["detect_action_latency"] = (d_time - s_time).total_seconds()
            #time_info["total_latency"] = (d_time - c_time).total_seconds()

            #dict_result["time_info"] = time_info


            # JSON出力用
            self.publish_data('/cs/data/sensor/detect_gesture/c_{}'.format(self.no), dict_result, sensing_time)

            # 標準出力
            # mess = 'Seat_ID: {} face_ward_relative:{:.2f}'.format(
                #  dict_result['body']['raw']['r_handtip']['x'])
            mess = 'push:{} R:{} L:{} shuriken:{} R:{} L:{} lights:{} R:{} L:{} clap:{}'.format(
                #  dict_result['status']['rule_action']['is_hand_swipe'],
                #  dict_result['status']['rule_action']['is_hand_swing'],
                dict_result['status']['rule_action']['is_hand_push'],
                dict_result['status']['rule_action']['is_r_hand_push'],
                dict_result['status']['rule_action']['is_l_hand_push'],
                #  dict_result['status']['rule_action']['is_hand_up'],
                #  dict_result['status']['rule_action']['is_hand_clap'],
                #  #dict_result['status']['rule_action']['is_hand_xy'],
                #  dict_result['status']['rule_action']['is_hand_x'],
                #  dict_result['status']['rule_action']['is_hand_y'],
                #  dict_result['status']['rule_action']['is_hand_z'])
                # dict_result['status']['rule_action']['is_throwseed'],
                # dict_result['status']['rule_action']['is_r_throwseed'],
                # dict_result['status']['rule_action']['is_l_throwseed'],
                dict_result['status']['rule_action']['throw_shuriken'],
                dict_result['status']['rule_action']['is_r_shuriken'],
                dict_result['status']['rule_action']['is_l_shuriken'],
                dict_result['status']['rule_action']['is_hand_lights'],
                dict_result['status']['rule_action']['is_r_hand_lights'],
                dict_result['status']['rule_action']['is_l_hand_lights'],
                dict_result['status']['rule_action']['is_hand_clap'])
            #mess = 'Seat_ID: {} Swipe:{} Swing:{} Push:{}'.format(
            #    char_id, dict_result['body']['raw']['l_handtip']['x'], dict_result['body']['raw']['l_handtip']['y'],dict_result['body']['raw']['l_handtip']['z'])
            # mess = 'Seat_ID: {} left:{} right:{}'.format(
                # char_id, dict_result['status']['hand_points']['left']['hand_point_x'], dict_result['status']['hand_points']['right']['hand_point_x'])
            out_box.append(mess)
            self.get_logger().info("%s" % out_box)
        except:
            self.get_logger().error("%s" % traceback.format_exc())
def main():
    args = []
    rclpy.init(args=args)

    node = CSDetectGesture()
    node.run()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
