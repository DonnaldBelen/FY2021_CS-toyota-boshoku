#!/usr/bin/env python
import numpy as np

class BodyChecker:
    def __init__(self,
                 axis=3,
                 joint=31,
                 threshold=0,
                 ):
        self.axis = axis
        self.joint = joint
        self.is_data_error = False
        self.convert_list = [
            ['pelvis', 0],
            ['naval', 1],
            ['chest', 2],
            ['neck', 3],
            ['l_clavicle', 4],
            ['l_shoulder', 5],
            ['l_elbow', 6],
            ['l_wrist', 7],
            ['l_hand', 8],
            ['l_handtip', 9],
            ['l_thumb', 10],
            ['r_clavicle', 11],
            ['r_shoulder', 12],
            ['r_elbow', 13],
            ['r_wrist', 14],
            ['r_hand', 15],
            ['r_handtip', 16],
            ['r_thumb', 17],
            ['l_hip', 18],
            ['l_knee', 19],
            ['l_ankle', 20],
            ['l_foot', 21],
            ['r_hip', 22],
            ['r_knee', 23],
            ['r_ankle', 24],
            ['r_foot', 25],
            ['head', 26],
            ['nose', 27],
            ['l_eyes', 28],
            ['l_ear', 29],
            ['r_eyes', 30],
            ['r_ear', 31],
        ]
        self.threshold = threshold

    def Set_threshold(self, threshold):
        self.threshold = threshold

    def Check_neck_knee_error(self, neck, l_knee, r_knee):
        d_body = np.max([l_knee[1], r_knee[1]])
        d_th = neck[1]
        is_data_error = (d_body > d_th)
        return d_body, is_data_error

    def Check_pelvis_pos_error(self, pelvis, threshold):
        d_body = pelvis[1]
        is_data_error = (d_body > threshold)
        return d_body, is_data_error

    def Check_neck_chest_error(self, chest, neck, head):
        d_body = neck[1] - chest[1]
        d_th = np.linalg.norm(head - neck, ord=2)
        is_data_error = (d_body < d_th)
        return d_body, is_data_error

    def CheckError(self, dict_data):
        # 浮遊チェック
        pelvis = self.get_joints(dict_data, joint=0)
        chest = self.get_joints(dict_data, joint=2)
        neck = self.get_joints(dict_data, joint=3)
        l_knee = self.get_joints(dict_data, joint=19)
        r_knee = self.get_joints(dict_data, joint=23)
        head = self.get_joints(dict_data, joint=26)
        # 判定
        box = []
        d_body, is_data_error = self.Check_neck_chest_error(
            chest, neck, head)
        self.d_neck_chest = d_body
        box.append(is_data_error)
        d_body, is_data_error = self.Check_neck_knee_error(
            neck, l_knee, r_knee)
        self.d_neck_knee = d_body
        box.append(is_data_error)
        d_body, is_data_error = self.Check_pelvis_pos_error(
            pelvis, self.threshold)
        self.d_pelvis_pos = d_body
        box.append(is_data_error)
        self.is_data_error = is_data_error
        # self.is_data_error = any(box)
        return self.is_data_error

    def get_joints(self, dict_data, joint=0):
        points = np.zeros((self.axis))
        jo = joint
        for ax in range(self.axis):
            point = dict_data['joints'][jo]['position'][ax]
            points[ax] = point
        return points
