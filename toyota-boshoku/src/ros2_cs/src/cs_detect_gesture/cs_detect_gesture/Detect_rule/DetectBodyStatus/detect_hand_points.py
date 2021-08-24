# coding:utf-8
import configparser
import json
import ast
import numpy as np
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from detect_cross_point import DetectCrossPoint

class DetectHandPoints:
    def __init__(self):
        self.axis = axis = 3
        self.zero = np.zeros((axis))

        body_conf = configparser.ConfigParser()
        body_conf.read(
            os.path.dirname(
                os.path.abspath(__file__)) +
            '/config/BodyStatusConfig.ini',
            'UTF-8')

        self.offset_h_dir = json.loads(
            body_conf.get('hand_config', 'offset_h_dir'))
        self.offset_v_dir = json.loads(
            body_conf.get('hand_config', 'offset_v_dir'))

        self.up_wide = json.loads(
            body_conf.get('hand_config', 'up_sight_wide'))
        self.down_wide = json.loads(
            body_conf.get('hand_config', 'down_sight_wide'))
        self.right_wide = json.loads(
            body_conf.get('hand_config', 'right_sight_wide'))
        self.left_wide = json.loads(
            body_conf.get('hand_config', 'left_sight_wide'))

        object_tank = ast.literal_eval(
            body_conf.get('cs_config', 'object_tank'))
        self.detect_cross = DetectCrossPoint(object_tank=object_tank)

        self.hand_point_left_x = 0.0
        self.hand_point_left_y = 0.0
        self.hand_point_left_z = 0.0
        self.hand_point_left_object = 'none'
        self.hand_point_left_distance = 0.0
        self.hand_direction_horizontal_left = 0.0
        self.hand_direction_vertical_left = 0.0
        self.hand_direction_bace_left_x = 0.0
        self.hand_direction_bace_left_y = 0.0
        self.hand_direction_bace_left_z = 0.0
        self.is_insight_left_hand = False

        self.hand_point_right_x = 0.0
        self.hand_point_right_y = 0.0
        self.hand_point_right_z = 0.0
        self.hand_point_right_object = 'none'
        self.hand_point_right_distance = 0.0
        self.hand_direction_horizontal_right = 0.0
        self.hand_direction_vertical_right = 0.0
        self.hand_direction_bace_right_x = 0.0
        self.hand_direction_bace_right_y = 0.0
        self.hand_direction_bace_right_z = 0.0
        self.is_insight_right_hand = False

    def set_input_data(self, body_status_dict):
        self.hand_direction_horizontal_left = body_status_dict['left_hand']['direction_horizontal']
        self.hand_direction_vertical_left =   body_status_dict['left_hand']['direction_vertical']
        self.hand_direction_bace_left_x =     body_status_dict['left_hand']['direction_bace_x']
        self.hand_direction_bace_left_y =     body_status_dict['left_hand']['direction_bace_y']
        self.hand_direction_bace_left_z =     body_status_dict['left_hand']['direction_bace_z']

        self.hand_direction_horizontal_right = body_status_dict['right_hand']['direction_horizontal']
        self.hand_direction_vertical_right =   body_status_dict['right_hand']['direction_vertical']
        self.hand_direction_bace_right_x =     body_status_dict['right_hand']['direction_bace_x']
        self.hand_direction_bace_right_y =     body_status_dict['right_hand']['direction_bace_y']
        self.hand_direction_bace_right_z =     body_status_dict['right_hand']['direction_bace_z']

        self.head_direction_horizontal = body_status_dict['head']['direction_horizontal']
        self.head_direction_vertical =   body_status_dict['head']['direction_vertical']
        self.head_direction_bace_x =     body_status_dict['head']['direction_bace_x']
        self.head_direction_bace_y =     body_status_dict['head']['direction_bace_y']
        self.head_direction_bace_z =     body_status_dict['head']['direction_bace_z']

    def detect_insight(self, part):
        is_flag = False
        up_lim = self.head_direction_vertical + self.up_wide
        down_lim = self.head_direction_vertical - self.down_wide
        left_lim = self.head_direction_horizontal + self.left_wide
        right_lim = self.head_direction_horizontal - self.right_wide
        if(part == 'right'):
            target_vertical = self.hand_direction_vertical_right
            target_horizontal = self.hand_direction_horizontal_right
        elif(part == 'left'):
            target_vertical = self.hand_direction_vertical_left
            target_horizontal = self.hand_direction_horizontal_left
        is_flag_vertical = up_lim > target_vertical and down_lim < target_vertical

        ifc = target_horizontal * self.head_direction_horizontal
        if(ifc < 0 and abs(target_horizontal) > 100 and abs(self.head_direction_horizontal) > 100):
            if(target_horizontal < 0):
                target_horizontal = target_horizontal + 360
            else:
                target_horizontal = target_horizontal - 360
        is_flag_horizontal = right_lim < target_horizontal and target_horizontal < left_lim
        if(is_flag_vertical and is_flag_horizontal):
            is_flag = True

        return is_flag

    def calculate_look_point(self):
        offset_h = self.offset_h_dir
        offset_v = self.offset_v_dir

        # left 
        h_dir = self.hand_direction_horizontal_left
        v_dir = self.hand_direction_vertical_left
        self.detect_cross.Calculate(h_dir=(h_dir + offset_h),
                                    v_dir=(v_dir + offset_v),
                                    bace_x=self.hand_direction_bace_left_x,
                                    bace_y=self.hand_direction_bace_left_y,
                                    bace_z=self.hand_direction_bace_left_z,)
        self.hand_point_left_x = self.detect_cross.look_point_x
        self.hand_point_left_y = self.detect_cross.look_point_y
        self.hand_point_left_z = self.detect_cross.look_point_z
        self.hand_point_left_object = self.detect_cross.look_object
        self.hand_point_left_distance = self.detect_cross.look_distance
        self.is_insight_left_hand = self.detect_insight('left')

        # right
        h_dir = self.hand_direction_horizontal_right
        v_dir = self.hand_direction_vertical_right
        self.detect_cross.Calculate(h_dir=(h_dir + offset_h),
                                    v_dir=(v_dir + offset_v),
                                    bace_x=self.hand_direction_bace_right_x,
                                    bace_y=self.hand_direction_bace_right_y,
                                    bace_z=self.hand_direction_bace_right_z,)
        self.hand_point_right_x = self.detect_cross.look_point_x
        self.hand_point_right_y = self.detect_cross.look_point_y
        self.hand_point_right_z = self.detect_cross.look_point_z
        self.hand_point_right_object = self.detect_cross.look_object
        self.hand_point_right_distance = self.detect_cross.look_distance
        self.is_insight_right_hand = self.detect_insight('right')

    def Calculate(self,
                  body_status_dict,
                  is_data=False):
        if (is_data):
            self.set_input_data(body_status_dict)
            self.calculate_look_point()
        else:
            self.hand_point_left_x = 0.0
            self.hand_point_left_y = 0.0
            self.hand_point_left_z = 0.0
            self.hand_point_left_object = 'none'
            self.hand_point_left_distance = 0.0
            self.is_insight_left_hand = False

            self.hand_point_right_x = 0.0
            self.hand_point_right_y = 0.0
            self.hand_point_right_z = 0.0
            self.hand_point_right_object = 'none'
            self.hand_point_right_distance = 0.0
            self.is_insight_right_hand = False

        output_dict = {}
        output_dict['left'] = {}
        left_dict = {}
        left_dict['hand_point_x'] = self.hand_point_left_x
        left_dict['hand_point_y'] = self.hand_point_left_y
        left_dict['hand_point_z'] = self.hand_point_left_z
        left_dict['hand_point_object'] = self.hand_point_left_object
        left_dict['hand_point_distance'] = self.hand_point_left_distance
        left_dict['is_insight'] = self.is_insight_left_hand
        output_dict['left'] = left_dict

        output_dict['right'] = {}
        right_dict = {}
        right_dict['hand_point_x'] = self.hand_point_right_x
        right_dict['hand_point_y'] = self.hand_point_right_y
        right_dict['hand_point_z'] = self.hand_point_right_z
        right_dict['hand_point_object'] = self.hand_point_right_object
        right_dict['hand_point_distance'] = self.hand_point_right_distance
        right_dict['is_insight'] = self.is_insight_right_hand
        output_dict['right'] = right_dict
        self.output_data = output_dict

    def set_offset(self, offset_h=0.0, offset_v=0.0):
        self.offset_h_dir = offset_h
        self.offset_v_dir = offset_v
