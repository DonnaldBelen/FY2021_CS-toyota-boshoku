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
import argparse

# 前処理用
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Detect_rule.detect_rule import Detect_rule
from Preprocessing.Preprocessor import Preprocessor

class CSDetectAction(CSNode):
    def __init__(self):
        super().__init__('CSDetectAction')

        parser = argparse.ArgumentParser(description='CSDetectAction')
        parser.add_argument('--no', type=str, default="") # NO
        args = parser.parse_args()

        self.no = args.no

        self.get_logger().info("CSDetectAction NO => {}".format(self.no))

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

            # JSON出力用
            self.publish_data('/cs/data/sensor/detect_action/c_{}'.format(self.no), dict_result, sensing_time)

            # 標準出力
            mess = 'Seat_ID: {} face_ward_relative:{:.2f}'.format(
                self.no, dict_result['status']['face_direction']['face_direction_horizontal'])

            out_box.append(mess)
            self.get_logger().info("%s" % out_box)
        
        except:
            self.get_logger().error("%s" % traceback.format_exc())

def main():
    args = []
    rclpy.init(args=args)

    node = CSDetectAction()
    node.run()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
