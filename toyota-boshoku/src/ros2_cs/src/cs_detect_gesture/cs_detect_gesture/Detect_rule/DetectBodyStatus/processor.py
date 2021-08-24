#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from detect_action import Detect_action
from detect_face_to_hand_direction_for_azure import DetectFaceToHandDirection
from detect_hand_points import DetectHandPoints

class Processor:
    def __init__(self):
        self.detect_action = Detect_action()
        self.face2hand = DetectFaceToHandDirection()
        self.cross_point = DetectHandPoints()

    def Calculate(self, data_dict, is_data=False):
        self.detect_action.Calculate(data_dict, is_data=is_data)
        rule_action_dict = self.detect_action.output_data

        self.face2hand.Calculate(data_dict, is_data=is_data)
        hand_direction_dict = self.face2hand.output_data

        self.cross_point.Calculate(hand_direction_dict, is_data=is_data)
        hand_points_dict = self.cross_point.output_data

        output_dict = {}
        output_dict['rule_action'] = rule_action_dict
        output_dict['hand_direction'] = hand_direction_dict
        output_dict['hand_points'] = hand_points_dict
        self.output_data = output_dict
