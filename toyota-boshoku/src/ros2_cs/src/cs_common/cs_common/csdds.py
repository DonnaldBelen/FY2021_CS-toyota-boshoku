#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Utility Classes for CS Environment
"""

from enum import IntEnum
from typing import TypeVar
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

NODE_TAG_PUB = 'cs_publisher'
NODE_TAG_SUB = 'cs_subscriber'

GenType = TypeVar('GenType')

class CSPublisherNode(Node):
    """ Publisher Node creation class """

    def __init__(self, topic, msg_type: GenType, qos_profile):
        self.cspub = super().__init__(NODE_TAG_PUB + topic.replace('/', '_'))
        self.publisher = self.create_publisher(msg_type, topic, qos_profile)

    def publish(self, message):
        """ Publish message """

        if isinstance(message, str):
            msg = String()
            msg.data = message
            self.publisher.publish(msg)
        elif isinstance(message, GenType):
            self.publisher.publish(message)

    def get_publisher(self):
        return self.publisher

class CSSubscriberNode(Node):
    """ Subscriber Node creation class """

    def __init__(self, topic, msg_type: GenType, qos_profile, callback):
        self.cssub = super().__init__(NODE_TAG_SUB + topic.replace('/', '_'))
        self.subscriber = self.create_subscription(msg_type, topic, callback, qos_profile)

    def get_subscriber(self):
        return self.subscriber

class CSQoS():
    """ QoSProfile creation class """

    def __init__(self) -> None:
        pass

    class QoSType(IntEnum):
        """ QoSProfile Type enum. """
        SENSOR = 1
        STREAM = 2
        PARAMETER = 3
        # Add QoSProfile name as appropriate after this

    @staticmethod
    def create(qos_type: QoSType) -> QoSProfile:
        """" create QoSProfile """

        if qos_type == CSQoS.QoSType.SENSOR:
            qos_profile_custom = QoSProfile(depth=1)
            qos_profile_custom.history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST
            qos_profile_custom.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
            qos_profile_custom.durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE

        elif qos_type == CSQoS.QoSType.STREAM:
            qos_profile_custom = QoSProfile(depth=1)
            qos_profile_custom.history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST
            qos_profile_custom.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
            qos_profile_custom.durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE

        elif qos_type == CSQoS.QoSType.PARAMETER:
            qos_profile_custom = QoSProfile(depth=1000)
            qos_profile_custom.history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL
            qos_profile_custom.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
            qos_profile_custom.durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE

        return qos_profile_custom

if __name__ == '__main__':
    pass
