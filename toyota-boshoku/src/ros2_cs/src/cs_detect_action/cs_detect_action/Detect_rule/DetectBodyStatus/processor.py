#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from detect_face_direction_for_azure import DetectFaceDirection
from detect_look_points import DetectLookPoints

class Processor:
    def __init__(self, ids='1'):
        self.face_direction = DetectFaceDirection()
        self.look_points = DetectLookPoints(ids=ids)

    def Calculate(self, data_dict, is_data=False):
        self.face_direction.Calculate(data_dict, is_data=is_data)
        face_direction_dict = self.face_direction.output_data

        self.look_points.Calculate(face_direction_dict, is_data=is_data)
        look_points_dict = self.look_points.output_data

        output_dict = {}
        output_dict['face_direction'] = face_direction_dict
        output_dict['look_points'] = look_points_dict
        self.output_data = output_dict
