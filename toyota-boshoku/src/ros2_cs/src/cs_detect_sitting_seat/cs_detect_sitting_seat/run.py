from datetime import date, datetime, timedelta
from cs_common.common import MyEncoder
from cs_common.csdds import CSPublisherNode
from cs_common.csdds import CSQoS
from cs_common.csdds import CSSubscriberNode
from cs_common.common import CSNode
import cs_common.common
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String
import configparser
import glob
import json
import os
import rclpy
import re
import subprocess
import sys
import time
import traceback
import argparse
import numpy as np

class CSDetectSittingSeat(CSNode):
    def __init__(self):
        super().__init__('CSDetectSittingSeat')

        parser = argparse.ArgumentParser(description='CSDetectSittingSeat')
        parser.add_argument('--no', type=str, default="0") # NO
        args = parser.parse_args()

        self.no = args.no

        self.get_logger().info("CSDetectSittingSeat NO => {}".format(self.no))

        self.THRESHOLD_DISTANCE = self.inifile.getint('seat_detection','Threshold_Seat_Distance')#240

        self.SEAT_POINT = self.inifile.get('seat_detection','SEAT_{}_POINT'.format(self.no))

        self.CS_x_max = self.inifile.getint('seat_detection','cs_area_x_1')
        self.CS_x_min = self.inifile.getint('seat_detection','cs_area_x_0')
        self.CS_z_max = self.inifile.getint('seat_detection','cs_area_z_1')
        self.CS_z_min = self.inifile.getint('seat_detection','cs_area_z_0')

        self.azure_linect_data_list = []

    def run(self):
        qos_profile = CSQoS.create(CSQoS.QoSType.SENSOR)
        
        subnode1 = CSSubscriberNode('/cs/data/sensor/azure_kinect/c_99',
            String, qos_profile, self.callback_subscribe0)
        
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(subnode1)
        executor.add_node(self)

        try:
            executor.spin()
        except KeyboardInterrupt:
            self.get_logger().warning("KeyboardInterrupt!")

        executor.shutdown()

        subnode1.destroy_node()

    def callback_subscribe0(self, msg):
        try:
            tmp = json.loads(msg.data)
            self.check_person_position(tmp, tmp['sensing_time'])
        except:
            self.get_logger().error("%s" % traceback.format_exc())
            
    def is_riding(self,merged_data): #NCP Edit
        riding=0
        CS_x_max = self.CS_x_max
        CS_x_min = self.CS_x_min
        CS_z_max = self.CS_z_max
        CS_z_min = self.CS_z_min
        for psi in range(len(merged_data["skeletons"])):
            if ((merged_data["skeletons"][psi]["joints"][cs_common.common.K4ABT_JOINT_SPINE_NAVAL]["position"][0] < CS_x_max) and (merged_data["skeletons"][psi]["joints"][cs_common.common.K4ABT_JOINT_SPINE_NAVAL]["position"][0] > CS_x_min) and (merged_data["skeletons"][psi]["joints"][cs_common.common.K4ABT_JOINT_SPINE_NAVAL]["position"][2] < CS_z_max) and (merged_data["skeletons"][psi]["joints"][cs_common.common.K4ABT_JOINT_SPINE_NAVAL]["position"][2] > CS_z_min)):
                riding += 1

        return riding

    def get_area_person(self, merged_data, compare_joint_num, point, threshold_distance):

        for psi in range(len(merged_data["skeletons"])):

            # 距離計算
            self.get_logger().info("%s" % merged_data["skeletons"][psi]["joints"][compare_joint_num]["position"])
            compare_pos = np.array(merged_data["skeletons"][psi]["joints"][compare_joint_num]["position"])
            distance = np.linalg.norm(point - compare_pos)

            # 一定以下かどうか
            if distance < threshold_distance:
                return merged_data["skeletons"][psi]

        return None

    def check_person_position(self, merged_data, sensing_time):
        COMPARE_JOINT_NUM = cs_common.common.K4ABT_JOINT_SPINE_NAVAL
        #COMPARE_JOINT_NUM = cs_common.common.K4ABT_JOINT_NECK
        CAPTURE_POINT = np.array([0 ,0, 0])
        THRESHOLD_DISTANCE = self.THRESHOLD_DISTANCE


        SEAT_POINT = np.asarray(json.loads(self.SEAT_POINT))

        sitting_data = {}

        seat = self.get_area_person(merged_data, COMPARE_JOINT_NUM, SEAT_POINT, THRESHOLD_DISTANCE)

        riding = self.is_riding(merged_data) #NCP Edit

        sitting_data["seat"] = seat
        sitting_data["riding"] = riding

        self.publish_data('/cs/data/sensor/sitting_seat_{}'.format(self.no), sitting_data, sensing_time)

def main():
    args = []
    rclpy.init(args=args)

    node = CSDetectSittingSeat()
    node.run()

    rclpy.shutdown()

if __name__ == '__main__':
    main()