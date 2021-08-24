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
import subprocess
import json
from datetime import date, datetime, timedelta
import re
import argparse

class CSControlVehicleContents(CSNode):
    def __init__(self):
        super().__init__('CSControlVehicleContents')

        parser = argparse.ArgumentParser(description='CSControlVehicleContents')
        parser.add_argument('--no', type=str, default="") # NO
        args = parser.parse_args()

        self.no = args.no

        self.get_logger().info("CSControlVehicleContents NO => {}".format(self.no))

    def run(self):
        qos_profile = CSQoS.create(CSQoS.QoSType.SENSOR)
        
        subnode1 = CSSubscriberNode('/cs/data/sensor/????_{}'.format(self.no),
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
        json_data = json.loads(msg.data)

        tic = time.time()

        # 入力データ取得
        sensing_time = json_data['sensing_time']
        dic_data = json_data
        
        # JSON出力用
        self.publish_data('/cs/data/sensor/?????/c_{}'.format(self.no), dict_result, sensing_time)

def main():
    args = []
    rclpy.init(args=args)

    node = CSControlVehicleContents()
    node.run()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
