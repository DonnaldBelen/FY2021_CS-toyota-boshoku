# coding:utf-8
import configparser
import json
import ast
import numpy as np
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from detect_cross_point import DetectCrossPoint

class DetectLookPoints:
    def __init__(self, ids='1'):
        self.axis = axis = 3
        self.zero = np.zeros((axis))

        body_conf = configparser.ConfigParser()
        body_conf.read(
            os.path.dirname(
                os.path.abspath(__file__)) +
            '/config/BodyStatusConfig.ini',
            'UTF-8')
        self.left_wide = json.loads(body_conf.get('body_config', 'left_wide'))
        self.right_wide = json.loads(body_conf.get('body_config', 'right_wide'))
        self.up_wide = json.loads(body_conf.get('body_config', 'up_wide'))
        self.down_wide = json.loads(body_conf.get('body_config', 'down_wide'))

        self.offset_h_dir = json.loads(
            body_conf.get('body_config', 'offset_h_dir'))
        self.offset_v_dir = json.loads(
            body_conf.get('body_config', 'offset_v_dir'))

        if(ids=='2'):
            object_tank = ast.literal_eval(
                body_conf.get('cs_config_S2', 'object_tank'))
            object_version = ast.literal_eval(
                body_conf.get('cs_config_S2', 'version'))
        elif(ids=='3'):
            object_tank = ast.literal_eval(
                body_conf.get('cs_config_S3', 'object_tank'))
            object_version = ast.literal_eval(
                body_conf.get('cs_config_S3', 'version'))
        else:
            object_tank = ast.literal_eval(
                body_conf.get('cs_config', 'object_tank'))
            object_version = ast.literal_eval(
                body_conf.get('cs_config', 'version'))
        self.detect_cross = DetectCrossPoint(object_tank=object_tank)
        self.object_version = object_version
        # self.ids = ids

        self.look_point_center_x = 0.0
        self.look_point_center_y = 0.0
        self.look_point_center_z = 0.0
        self.look_point_center_object = 'none'
        self.look_point_center_distance = 0.0

        self.look_point_left_eyes_x = 0.0
        self.look_point_left_eyes_y = 0.0
        self.look_point_left_eyes_z = 0.0
        self.look_point_left_eyes_object = 'none'
        self.look_point_left_eyes_distance = 0.0

        self.look_point_right_eyes_x = 0.0
        self.look_point_right_eyes_y = 0.0
        self.look_point_right_eyes_z = 0.0
        self.look_point_right_eyes_object = 'none'
        self.look_point_right_eyes_distance = 0.0

        self.look_point_left_x = 0.0
        self.look_point_left_y = 0.0
        self.look_point_left_z = 0.0
        self.look_point_left_object = 'none'
        self.look_point_left_distance = 0.0

        self.look_point_right_x = 0.0
        self.look_point_right_y = 0.0
        self.look_point_right_z = 0.0
        self.look_point_right_object = 'none'
        self.look_point_right_distance = 0.0

        self.look_point_top_x = 0.0
        self.look_point_top_y = 0.0
        self.look_point_top_z = 0.0
        self.look_point_top_object = 'none'
        self.look_point_top_distance = 0.0

        self.look_point_bottom_x = 0.0
        self.look_point_bottom_y = 0.0
        self.look_point_bottom_z = 0.0
        self.look_point_bottom_object = 'none'
        self.look_point_bottom_distance = 0.0

        self.face_direction_horizontal = 0.0
        self.face_direction_vertical = 0.0
        self.face_direction_bace_x = 0.0
        self.face_direction_bace_y = 0.0
        self.face_direction_bace_z = 0.0
        self.face_direction_right_eyes_x = 0.0
        self.face_direction_right_eyes_y = 0.0
        self.face_direction_right_eyes_z = 0.0
        self.face_direction_left_eyes_x = 0.0
        self.face_direction_left_eyes_y = 0.0
        self.face_direction_left_eyes_z = 0.0

    def set_input_data(self, body_status_dict):
        self.face_direction_horizontal = body_status_dict['face_direction_horizontal']
        self.face_direction_vertical = body_status_dict['face_direction_vertical']
        self.face_direction_bace_x = body_status_dict['face_direction_bace_x']
        self.face_direction_bace_y = body_status_dict['face_direction_bace_y']
        self.face_direction_bace_z = body_status_dict['face_direction_bace_z']
        self.face_direction_right_eyes_x = body_status_dict['face_direction_bace_right_eyes_x']
        self.face_direction_right_eyes_y = body_status_dict['face_direction_bace_right_eyes_y']
        self.face_direction_right_eyes_z = body_status_dict['face_direction_bace_right_eyes_z']
        self.face_direction_left_eyes_x = body_status_dict['face_direction_bace_left_eyes_x']
        self.face_direction_left_eyes_y = body_status_dict['face_direction_bace_left_eyes_y']
        self.face_direction_left_eyes_z = body_status_dict['face_direction_bace_left_eyes_z']

    def calculate_look_wide(self):
        h_dir = self.face_direction_horizontal
        if(h_dir > 180):
            h_dir = h_dir - 360
        if(h_dir < -180):
            h_dir = h_dir + 360
        v_dir = self.face_direction_vertical
        if(v_dir > 180):
            v_dir = v_dir - 360
        if(v_dir < -180):
            v_dir = v_dir + 360
        b_x = self.face_direction_bace_x
        b_y = self.face_direction_bace_y
        b_z = self.face_direction_bace_z
        r_x = self.face_direction_right_eyes_x
        r_y = self.face_direction_right_eyes_y
        r_z = self.face_direction_right_eyes_z
        l_x = self.face_direction_left_eyes_x
        l_y = self.face_direction_left_eyes_y
        l_z = self.face_direction_left_eyes_z

        offset_h = self.offset_h_dir
        offset_v = self.offset_v_dir

        # left 
        add_left = self.left_wide
        if(abs(h_dir) >= 90):
            add_left = self.left_wide *(-1)
        self.detect_cross.Calculate(h_dir=(h_dir + add_left + offset_h),
                                    v_dir=(v_dir + offset_v),
                                    bace_x=l_x,
                                    bace_y=l_y,
                                    bace_z=l_z,)
        self.look_point_left_x = self.detect_cross.look_point_x
        self.look_point_left_y = self.detect_cross.look_point_y
        self.look_point_left_z = self.detect_cross.look_point_z
        self.look_point_left_object = self.detect_cross.look_object
        self.look_point_left_distance = self.detect_cross.look_distance

        self.detect_cross.Calculate(h_dir=(h_dir + offset_h),
                                    v_dir=(v_dir + offset_v),
                                    bace_x=l_x,
                                    bace_y=l_y,
                                    bace_z=l_z,)
        self.look_point_left_eyes_x = self.detect_cross.look_point_x
        self.look_point_left_eyes_y = self.detect_cross.look_point_y
        self.look_point_left_eyes_z = self.detect_cross.look_point_z
        self.look_point_left_eyes_object = self.detect_cross.look_object
        self.look_point_left_eyes_distance = self.detect_cross.look_distance

        self.detect_cross.Calculate(h_dir=(h_dir + offset_h),
                                    v_dir=(v_dir + offset_v),
                                    bace_x=b_x,
                                    bace_y=b_y,
                                    bace_z=b_z,)
        self.look_point_center_x = self.detect_cross.look_point_x
        self.look_point_center_y = self.detect_cross.look_point_y
        self.look_point_center_z = self.detect_cross.look_point_z
        self.look_point_center_object = self.detect_cross.look_object
        self.look_point_center_distance = self.detect_cross.look_distance

        self.detect_cross.Calculate(h_dir=(h_dir + offset_h),
                                    v_dir=(v_dir + offset_v),
                                    bace_x=r_x,
                                    bace_y=r_y,
                                    bace_z=r_z,)
        self.look_point_right_eyes_x = self.detect_cross.look_point_x
        self.look_point_right_eyes_y = self.detect_cross.look_point_y
        self.look_point_right_eyes_z = self.detect_cross.look_point_z
        self.look_point_right_eyes_object = self.detect_cross.look_object
        self.look_point_right_eyes_distance = self.detect_cross.look_distance

        # right
        add_right = self.right_wide
        if(abs(h_dir) >= 90):
            add_right = self.right_wide * (-1)
        self.detect_cross.Calculate(h_dir=(h_dir - add_right + offset_h),
                                    v_dir=(v_dir + offset_v),
                                    bace_x=r_x,
                                    bace_y=r_y,
                                    bace_z=r_z,)
        self.look_point_right_x = self.detect_cross.look_point_x
        self.look_point_right_y = self.detect_cross.look_point_y
        self.look_point_right_z = self.detect_cross.look_point_z
        self.look_point_right_object = self.detect_cross.look_object
        self.look_point_right_distance = self.detect_cross.look_distance

        self.detect_cross.Calculate(h_dir=(h_dir + offset_h),
                                    v_dir=(v_dir + self.up_wide + offset_v),
                                    bace_x=b_x,
                                    bace_y=b_y,
                                    bace_z=b_z,)
        self.look_point_top_x = self.detect_cross.look_point_x
        self.look_point_top_y = self.detect_cross.look_point_y
        self.look_point_top_z = self.detect_cross.look_point_z
        self.look_point_top_object = self.detect_cross.look_object
        self.look_point_top_distance = self.detect_cross.look_distance

        self.detect_cross.Calculate(h_dir=(h_dir + offset_h),
                                    v_dir=(v_dir - self.down_wide + offset_v),
                                    bace_x=b_x,
                                    bace_y=b_y,
                                    bace_z=b_z,)
        self.look_point_bottom_x = self.detect_cross.look_point_x
        self.look_point_bottom_y = self.detect_cross.look_point_y
        self.look_point_bottom_z = self.detect_cross.look_point_z
        self.look_point_bottom_object = self.detect_cross.look_object
        self.look_point_bottom_distance = self.detect_cross.look_distance

    def Calculate(self,
                  body_status_dict,
                  is_data=False):
        if (is_data):
            self.set_input_data(body_status_dict)
            self.calculate_look_wide()
        else:
            self.look_point_center_x = 0.0
            self.look_point_center_y = 0.0
            self.look_point_center_z = 0.0
            self.look_point_center_object = 'none'
            self.look_point_center_distance = 0.0

            self.look_point_left_eyes_x = 0.0
            self.look_point_left_eyes_y = 0.0
            self.look_point_left_eyes_z = 0.0
            self.look_point_left_eyes_object = 'none'
            self.look_point_left_eyes_distance = 0.0

            self.look_point_right_eyes_x = 0.0
            self.look_point_right_eyes_y = 0.0
            self.look_point_right_eyes_z = 0.0
            self.look_point_right_eyes_object = 'none'
            self.look_point_right_eyes_distance = 0.0

            self.look_point_left_x = 0.0
            self.look_point_left_y = 0.0
            self.look_point_left_z = 0.0
            self.look_point_left_object = 'none'
            self.look_point_left_distance = 0.0

            self.look_point_right_x = 0.0
            self.look_point_right_y = 0.0
            self.look_point_right_z = 0.0
            self.look_point_right_object = 'none'
            self.look_point_right_distance = 0.0

            self.look_point_top_x = 0.0
            self.look_point_top_y = 0.0
            self.look_point_top_z = 0.0
            self.look_point_top_object = 'none'
            self.look_point_top_distance = 0.0

            self.look_point_bottom_x = 0.0
            self.look_point_bottom_y = 0.0
            self.look_point_bottom_z = 0.0
            self.look_point_bottom_object = 'none'
            self.look_point_bottom_distance = 0.0

        output_dict = {}
        output_dict['look_point_center_x'] = self.look_point_center_x
        output_dict['look_point_center_y'] = self.look_point_center_y
        output_dict['look_point_center_z'] = self.look_point_center_z
        output_dict['look_point_center_object'] = self.look_point_center_object
        output_dict['look_point_center_distance'] = self.look_point_center_distance

        output_dict['look_point_left_eyes_x'] = self.look_point_left_eyes_x
        output_dict['look_point_left_eyes_y'] = self.look_point_left_eyes_y
        output_dict['look_point_left_eyes_z'] = self.look_point_left_eyes_z
        output_dict['look_point_left_eyes_object'] = self.look_point_left_eyes_object
        output_dict['look_point_left_eyes_distance'] = self.look_point_left_eyes_distance

        output_dict['look_point_right_eyes_x'] = self.look_point_right_eyes_x
        output_dict['look_point_right_eyes_y'] = self.look_point_right_eyes_y
        output_dict['look_point_right_eyes_z'] = self.look_point_right_eyes_z
        output_dict['look_point_right_eyes_object'] = self.look_point_right_eyes_object
        output_dict['look_point_right_eyes_distance'] = self.look_point_right_eyes_distance

        output_dict['look_point_left_x'] = self.look_point_left_x
        output_dict['look_point_left_y'] = self.look_point_left_y
        output_dict['look_point_left_z'] = self.look_point_left_z
        output_dict['look_point_left_object'] = self.look_point_left_object
        output_dict['look_point_left_distance'] = self.look_point_left_distance

        output_dict['look_point_right_x'] = self.look_point_right_x
        output_dict['look_point_right_y'] = self.look_point_right_y
        output_dict['look_point_right_z'] = self.look_point_right_z
        output_dict['look_point_right_object'] = self.look_point_right_object
        output_dict['look_point_right_distance'] = self.look_point_right_distance

        output_dict['look_point_top_x'] = self.look_point_top_x
        output_dict['look_point_top_y'] = self.look_point_top_y
        output_dict['look_point_top_z'] = self.look_point_top_z
        output_dict['look_point_top_object'] = self.look_point_top_object
        output_dict['look_point_top_distance'] = self.look_point_top_distance

        output_dict['look_point_bottom_x'] = self.look_point_bottom_x
        output_dict['look_point_bottom_y'] = self.look_point_bottom_y
        output_dict['look_point_bottom_z'] = self.look_point_bottom_z
        output_dict['look_point_bottom_object'] = self.look_point_bottom_object
        output_dict['look_point_bottom_distance'] = self.look_point_bottom_distance

        output_dict['look_point_object_version'] = self.object_version
        # output_dict['ids'] = self.ids

        self.output_data = output_dict

    def set_offset(self, offset_h=0.0, offset_v=0.0):
        self.offset_h_dir = offset_h
        self.offset_v_dir = offset_v
