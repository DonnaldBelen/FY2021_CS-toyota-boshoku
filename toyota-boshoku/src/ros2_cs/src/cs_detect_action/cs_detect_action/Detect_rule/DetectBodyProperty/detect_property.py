#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import numpy as np

# 行動推定用ルール
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from property_height import Property_hight
from property_children import Property_children
from property_children2 import Property_children2

class Detect_property:
    def __init__(self, axis=3):
        # set parameter
        self.axis = axis
        self.axis_tank = ['x', 'y', 'z']
        self.is_data = False
        # set data
        self.pelvis = np.zeros((axis))
        self.naval = np.zeros((axis))
        self.chest = np.zeros((axis))
        self.neck = np.zeros((axis))
        self.l_clavicle = np.zeros((axis))
        self.r_clavicle = np.zeros((axis))
        self.l_shoulder = np.zeros((axis))
        self.r_shoulder = np.zeros((axis))
        self.l_elbow = np.zeros((axis))
        self.r_elbow = np.zeros((axis))
        self.l_wrist = np.zeros((axis))
        self.r_wrist = np.zeros((axis))
        self.l_hip = np.zeros((axis))
        self.r_hip = np.zeros((axis))
        self.l_knee = np.zeros((axis))
        self.r_knee = np.zeros((axis))
        self.l_ankle = np.zeros((axis))
        self.r_ankle = np.zeros((axis))
        self.l_foot = np.zeros((axis))
        self.r_foot = np.zeros((axis))
        self.head = np.zeros((axis))
        self.nose = np.zeros((axis))
        self.l_eyes = np.zeros((axis))
        self.r_eyes = np.zeros((axis))
        self.l_ear = np.zeros((axis))
        self.r_ear = np.zeros((axis))
        self.l_hand = np.zeros((axis))
        self.r_hand = np.zeros((axis))
        self.l_handtip = np.zeros((axis))
        self.r_handtip = np.zeros((axis))
        self.l_thumb = np.zeros((axis))
        self.r_thumb = np.zeros((axis))
        # インスタンス生成
        self.property_hight = Property_hight()
        self.property_children = Property_children()
        self.property_children2 = Property_children2()
        #
        self.output_data = {}
        self.is_children = False
        self.is_children2 = False

    def Update(self, body_dict):
        axt = self.axis_tank
        for ax in range(self.axis):
            self.l_ear[ax] = body_dict['l_ear'][axt[ax]]
            self.r_ear[ax] = body_dict['r_ear'][axt[ax]]
            self.l_shoulder[ax] = body_dict['l_shoulder'][axt[ax]]
            self.r_shoulder[ax] = body_dict['r_shoulder'][axt[ax]]
            self.l_elbow[ax] = body_dict['l_elbow'][axt[ax]]
            self.r_elbow[ax] = body_dict['r_elbow'][axt[ax]]
            self.l_hand[ax] = body_dict['l_hand'][axt[ax]]
            self.r_hand[ax] = body_dict['r_hand'][axt[ax]]
            self.l_wrist[ax] = body_dict['l_wrist'][axt[ax]]
            self.r_wrist[ax] = body_dict['r_wrist'][axt[ax]]
            self.l_handtip[ax] = body_dict['l_handtip'][axt[ax]]
            self.r_handtip[ax] = body_dict['r_handtip'][axt[ax]]
            self.neck[ax] = body_dict['neck'][axt[ax]]
            self.head[ax] = body_dict['head'][axt[ax]]

    def set_data(self,):
        # 出力データ準備
        dic_data = {}
        # 手を振る
        dic_data['height'] = int(self.height)
        dic_data['is_children'] = int(self.is_children)
        dic_data['is_children2'] = int(self.is_children2)
        dic_data['head_shoulder_width_ratio'] = self.ratio
        self.output_data = dic_data

    def Calculate(self, body_dict, is_data=False):
        # データ無しならFalse
        self.is_data = is_data
        if(self.is_data):
            # 骨格データの展開
            self.Update(body_dict)
            # 計算
            self.height = self.property_hight.calculate(
                l_shoulder=self.l_shoulder,
                r_shoulder=self.r_shoulder,
                l_elbow=self.l_elbow,
                r_elbow=self.r_elbow,
                l_hand=self.l_hand,
                r_hand=self.r_hand,
                l_wrist=self.l_wrist,
                r_wrist=self.r_wrist,
                l_handtip=self.l_handtip,
                r_handtip=self.r_handtip,
                is_data=self.is_data)
            self.is_children = self.property_children.calculate(
                height=self.height,
                is_data=self.is_data)
            self.is_children2 = self.property_children2.calculate(
                l_ear=self.l_ear,
                r_ear=self.r_ear,
                l_shoulder=self.l_shoulder,
                r_shoulder=self.r_shoulder,
                is_data=self.is_data)
            self.ratio = self.property_children2.ratio
        else:
            self.height = 0.0
            self.is_children = False
            self.is_children2 = False
            self.ratio = 0.0
        # データ格納
        self.set_data()
