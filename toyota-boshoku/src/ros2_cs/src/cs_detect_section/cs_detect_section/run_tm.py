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

class CSDetectSegment(CSNode):
    def __init__(self):
        super().__init__('CSDetectSegment')

        parser = argparse.ArgumentParser(description='CSDetectSegment')
        parser.add_argument('--no', type=str, default="3") # NO
        args = parser.parse_args()

        self.no = args.no

        self.get_logger().info("CSDetectSegment NO => {}".format(self.no))

        self.vertex_x1 = [514, 110, -294, -698]
        self.vertex_y1 = [847, 619, 391, 163]
        self.vertex_x2 = [611, 203, -205, -615]
        self.vertex_y2 = [813, 580, 347, 113]

    def run(self):
        qos_profile = CSQoS.create(CSQoS.QoSType.SENSOR)

        subnode1 = CSSubscriberNode('/cs/data/sensor/detect_action/c_{}'.format(self.no),
            String, qos_profile, self.callback_subscribe0)
        subnode2 = CSSubscriberNode('/cs/data/sensor/detect_gesture/c_{}'.format(self.no),
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
        try:
            json_data = json.loads(msg.data)
            char_id = '2'+self.no

            sensing_time = json_data['sensing_time']
            dic_tmp = json_data['status']['look_points']

            dic_data = {}
            list_axis = ['x','y','z','object']
            for i in range(len(list_axis)):
                axis = list_axis[i]
                dic_data['%s'%axis] = dic_tmp['look_point_center_%s'%axis]

            dict_result = None
            status = None
            dict_result = self.detect_section(status,dic_data)
            result = {
                    'res': dict_result,
                    'coord1':dic_data['x'],
                    'coord2':dic_data['y'],
                    'coord3':dic_data['z']
            }

            self.publish_data('/cs/data/sensor/detect_action/disp_location_{}'.format(self.no), result, sensing_time)
        except:
            self.get_logger().error("%s" % traceback.format_exc())

    def callback_subscribe1(self, msg):
        try:
            json_data = json.loads(msg.data)
            char_id = '3'+self.no

            sensing_time = json_data['sensing_time']
            dic_tmp_l = json_data['status']['hand_points']['left']
            dic_tmp_r = json_data['status']['hand_points']['right']

            is_r_hand_swing = json_data['status']['rule_action']['is_r_hand_swing']
            is_l_hand_swing = json_data['status']['rule_action']['is_l_hand_swing']

            dic_data_r = dic_data_l = {}
            list_axis = ['x','y','z','object']
            for i in range(len(list_axis)):
                axis = list_axis[i]
                dic_data_r['%s'%axis] = dic_tmp_r['hand_point_%s'%axis]
                dic_data_l['%s'%axis] = dic_tmp_l['hand_point_%s'%axis]

            dict_result_r = dict_result_l = None
            dict_result_r = self.detect_section(is_r_hand_swing,dic_data_r)
            dict_result_l = self.detect_section(is_l_hand_swing,dic_data_l)

            result = {}
            sensing_time = str(datetime.now())
            result = {
                'r_hand':dict_result_r,
                'l_hand':dict_result_l
            }
            self.publish_data('/cs/data/sensor/detect_gesture/disp_location_{}'.format(self.no), result, sensing_time)
        except:
            self.get_logger().error("%s" % traceback.format_exc())

    def detect_section(self,status,dic_data):
        cp_x = dic_data['x']
        cp_y = dic_data['y']
        cp_z = dic_data['z']
        cp_obj = dic_data['object']

        result = {}

        if cp_obj == "side_screen":
            for j in range(len(self.vertex_y1)-1):
                if cp_y < self.vertex_y1[j] and cp_y > self.vertex_y1[j+1]:
                    for i in range(len(self.vertex_x1)-1):
                        if cp_x < self.vertex_x1[i] and cp_x > self.vertex_x1[i+1]:
                            section = {
                                'i':i,
                                'j':j
                            }
                            x_order = section['i'] + 1
                            y_order = section['j'] * 3
                            result = x_order + y_order
        return result

        if cp_obj == "back_screen":
            for j in range(len(self.vertex_y2)-1):
                if cp_y < self.vertex_y2[j] and cp_y > self.vertex_y2[j+1]:
                    for i in range(len(self.vertex_x2)-1):
                        if cp_z > self.vertex_x2[i] and cp_z < self.vertex_x2[i+1]:
                            section = {
                                'i':i,
                                'j':j
                            }
                            x_order = section['i'] + 1
                            y_order = section['j'] * 3
                            result = x_order + y_order + 9
                            
        return result

def main():

    args = []
    rclpy.init(args=args)

    node = CSDetectSegment()
    node.run()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
