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
import cs_common.common as common
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

class CSDetectLeanBack(CSNode):
    def __init__(self):
        super().__init__('CSDetectLeanBack')

        self.THRESHOLD_BR_DISTANCE = self.inifile.getint('back_rest_pos','Threshold_Br_Distance')
        self.BR_1_POINT = self.inifile.get('back_rest_pos','BR_1_POINT')
        self.BR_2_POINT = self.inifile.get('back_rest_pos','BR_2_POINT')
        self.BR_3_POINT = self.inifile.get('back_rest_pos','BR_3_POINT')

        self.THRESHOLD_DISTANCE = self.inifile.getint('seat_detection','Threshold_Seat_Distance') 
        self.SEAT_1_POINT = self.inifile.get('seat_detection','SEAT_1_POINT')
        self.SEAT_2_POINT = self.inifile.get('seat_detection','SEAT_2_POINT')
        self.SEAT_3_POINT = self.inifile.get('seat_detection','SEAT_3_POINT')

    def run(self):
        qos_profile = CSQoS.create(CSQoS.QoSType.SENSOR)
        subnode = CSSubscriberNode('/cs/data/sensor/azure_kinect/c_99',
            String, qos_profile, self.callback_subscribe)

        try:
            rclpy.spin(subnode)
        except KeyboardInterrupt:
            self.get_logger().warning("KeyboardInterrupt!")

        subnode.destroy_node()

    def callback_subscribe(self, msg):
        try:
            json_data = json.loads(msg.data)
            sensing_time = json_data['sensing_time']
            self.is_lean_back(json_data, sensing_time)
        except:
            self.get_logger().error("%s" % traceback.format_exc())
            
    def is_lean_back(self, data, sensing_time):


        COMPARE_JOINT_NUM = common.K4ABT_JOINT_SPINE_NAVAL
        THRESHOLD_DISTANCE = self.THRESHOLD_DISTANCE

        SEAT_1_POINT = np.asarray(json.loads(self.SEAT_1_POINT))
        SEAT_2_POINT = np.asarray(json.loads(self.SEAT_2_POINT))
        SEAT_3_POINT = np.asarray(json.loads(self.SEAT_3_POINT))

        seat1 = self.get_area_person(data, COMPARE_JOINT_NUM, SEAT_1_POINT, THRESHOLD_DISTANCE)
        seat2 = self.get_area_person(data, COMPARE_JOINT_NUM, SEAT_2_POINT, THRESHOLD_DISTANCE)
        seat3 = self.get_area_person(data, COMPARE_JOINT_NUM, SEAT_3_POINT, THRESHOLD_DISTANCE)


        COMPARE_JOINT_BR_NUM = common.K4ABT_JOINT_SPINE_CHEST
        THRESHOLD_BR_DISTANCE = self.THRESHOLD_BR_DISTANCE

        BR_1_POINT = np.asarray(json.loads(self.BR_1_POINT))
        BR_2_POINT = np.asarray(json.loads(self.BR_2_POINT))
        BR_3_POINT = np.asarray(json.loads(self.BR_3_POINT))

        sitting_br_data = {
        }

        seat_br1 = self.get_area_person(data, COMPARE_JOINT_BR_NUM, BR_1_POINT, THRESHOLD_BR_DISTANCE)
        seat_br2 = self.get_area_person(data, COMPARE_JOINT_BR_NUM, BR_2_POINT, THRESHOLD_BR_DISTANCE)
        seat_br3 = self.get_area_person(data, COMPARE_JOINT_BR_NUM, BR_3_POINT, THRESHOLD_BR_DISTANCE)


        seat_list = [seat1, seat2, seat3]
        seatbr_list = [seat_br1,seat_br2,seat_br3]
        self.seatst_list = [0,0,0]

        for i in range(len(seat_list)):
            if seat_list[i] is not None:
                self.seatst_list[i] = 1
                
                if seatbr_list[i] is not None:
                    self.seatst_list[i] = 2
                
            elif seat_list[i] is None:
                self.seatst_list[i] = 0

        sitting_br_data["br_1"] = self.seatst_list[0]
        sitting_br_data["br_2"] = self.seatst_list[1]
        sitting_br_data["br_3"] = self.seatst_list[2]
            
        self.publish_data('/cs/data/sensor/lean_back', sitting_br_data, sensing_time)
        self.get_logger().info("%s" % sitting_br_data)


    def get_area_person(self, data, compare_joint_num, point, threshold_distance):
        for psi in range(len(data["skeletons"])):

            # 距離計算
            compare_pos = np.array(data["skeletons"][psi]["joints"][compare_joint_num]["position"])
            distance = np.linalg.norm(point - compare_pos)

            # 一定以下かどうか
            if distance < threshold_distance:
                return 1

def main():
    args = []
    rclpy.init(args=args)

    node = CSDetectLeanBack()
    node.run()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
