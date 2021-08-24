#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import numpy as np

# 行動推定用ルール
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from act_hand_swing import Act_Hand_Swing
from act_hand_swipe import Act_Hand_Swipe
from act_hand_push import Act_Hand_Push
from act_hand_up import Act_Hand_Up
from act_hand_clap import Act_Hand_Clap
from act_hand_stat import Act_Hand_Stat
from act_hand_point import Act_Hand_Point
from act_hand_throw_seed import Act_Hand_Throw_Seed
from act_hand_lights import Act_Hand_Lights
from act_hand_shuriken import Act_Hand_Shuriken

class Detect_action:
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
        self.act_hand_swing = Act_Hand_Swing()
        self.act_hand_swipe = Act_Hand_Swipe()
        self.act_hand_push = Act_Hand_Push()
        self.act_hand_up = Act_Hand_Up()
        self.act_hand_clap = Act_Hand_Clap()
        self.act_hand_stat = Act_Hand_Stat()
        self.act_hand_point = Act_Hand_Point()
        self.act_hand_throw_seed = Act_Hand_Throw_Seed()
        self.act_hand_shuriken = Act_Hand_Shuriken()
        self.act_hand_lights = Act_Hand_Lights()
        self.output_data={}

    def Update(self, body_dict):
        axt = self.axis_tank
        for ax in range(self.axis):
            self.l_elbow[ax] = body_dict['l_elbow'][axt[ax]]
            self.l_wrist[ax] = body_dict['l_wrist'][axt[ax]]
            self.r_elbow[ax] = body_dict['r_elbow'][axt[ax]]
            self.r_wrist[ax] = body_dict['r_wrist'][axt[ax]]
            self.r_hand[ax] = body_dict['r_hand'][axt[ax]]
            self.r_handtip[ax] = body_dict['r_handtip'][axt[ax]]
            self.r_shoulder[ax] = body_dict['r_shoulder'][axt[ax]]
            self.l_shoulder[ax] = body_dict['l_shoulder'][axt[ax]]
            self.l_hand[ax] = body_dict['l_hand'][axt[ax]]
            self.l_handtip[ax] = body_dict['l_handtip'][axt[ax]]
            self.head[ax] = body_dict['head'][axt[ax]]
            self.naval[ax] = body_dict['naval'][axt[ax]]
            self.chest[ax] = body_dict['chest'][axt[ax]]
            self.nose[ax] = body_dict['nose'][axt[ax]]

    def set_data(self,):
        # 出力データ準備
        dic_data = {}
        # 手を振る
        dic_data['is_hand_swing'] = int(self.is_hand_swing)
        dic_data['is_r_hand_swing'] = int(self.is_r_hand_swing)
        dic_data['is_l_hand_swing'] = int(self.is_l_hand_swing)
        dic_data['is_hand_swipe'] = int(self.is_hand_swipe)
        dic_data['is_r_hand_swipe'] = int(self.is_r_hand_swipe)
        dic_data['is_l_hand_swipe'] = int(self.is_l_hand_swipe)
        dic_data['is_hand_push'] = int(self.is_hand_push)
        dic_data['is_r_hand_push'] = int(self.is_r_hand_push)
        dic_data['is_l_hand_push'] = int(self.is_l_hand_push)
        dic_data['is_hand_up'] = int(self.is_hand_up)
        dic_data['is_l_hand_up'] = int(self.is_l_hand_up)
        dic_data['is_r_hand_up'] = int(self.is_r_hand_up)
        dic_data['is_hand_clap'] = int(self.is_hand_clap)
        dic_data['is_r_hand_clap'] = int(self.is_r_hand_clap)
        dic_data['is_l_hand_clap'] = int(self.is_l_hand_clap)
        dic_data['is_hand_x'] = int(self.is_hand_x)
        dic_data['is_hand_y'] = int(self.is_hand_y)
        dic_data['is_hand_z'] = int(self.is_hand_z)
        dic_data['is_screen_xy'] = int(self.is_screen_xy)
        dic_data['is_screen_x'] = int(self.is_screen_x)
        dic_data['is_screen_y'] = int(self.is_screen_y)
        dic_data['is_throwseed'] = int(self.is_throwseed)
        dic_data['is_r_throwseed'] = int(self.is_r_throwseed)
        dic_data['is_l_throwseed'] = int(self.is_l_throwseed)
        dic_data['throw_shuriken'] = int(self.throw_shuriken)
        dic_data['is_r_shuriken'] = int(self.is_r_shuriken)
        dic_data['is_l_shuriken'] = int(self.is_l_shuriken)
        dic_data['is_hand_lights'] = int(self.is_hand_lights)
        dic_data['is_r_hand_lights'] = int(self.is_r_hand_lights)
        dic_data['is_l_hand_lights'] = int(self.is_l_hand_lights)

        self.output_data = dic_data

    def Calculate(self, body_dict, is_data=False):
        # データ無しならFalse
        self.is_data = is_data
        if(self.is_data):
            # 骨格データの展開
            self.Update(body_dict)
            # 計算
            # 手を振る
            self.is_hand_swing, self.is_r_hand_swing, self.is_l_hand_swing = \
                self.act_hand_swing.calculate(
                    r_wrist=self.r_wrist,
                    l_wrist=self.l_wrist,
                    r_elbow=self.r_elbow,
                    l_elbow=self.l_elbow,
                    r_handtip=self.r_handtip,
                    l_handtip=self.l_handtip,
                    r_shoulder=self.r_shoulder,
                    l_shoulder=self.l_shoulder,
                    r_hand=self.r_hand,
                    l_hand=self.l_hand,
                    head=self.head,
                    naval=self.naval,
                    chest=self.chest,
                    is_data=self.is_data)
            self.is_hand_swipe, self.is_r_hand_swipe, self.is_l_hand_swipe = \
                self.act_hand_swipe.calculate(
                    r_wrist=self.r_wrist,
                    l_wrist=self.l_wrist,
                    r_elbow=self.r_elbow,
                    l_elbow=self.l_elbow,
                    r_handtip=self.r_handtip,
                    l_handtip=self.l_handtip,
                    r_shoulder=self.r_shoulder,
                    l_shoulder=self.l_shoulder,
                    r_hand=self.r_hand,
                    l_hand=self.l_hand,
                    head=self.head,
                    chest=self.chest,
                    naval=self.naval,
                    is_data=self.is_data)
            self.is_hand_push, self.is_r_hand_push, self.is_l_hand_push = \
                self.act_hand_push.calculate(
                    r_wrist=self.r_wrist,
                    l_wrist=self.l_wrist,
                    r_elbow=self.r_elbow,
                    l_elbow=self.l_elbow,
                    r_handtip=self.r_handtip,
                    l_handtip=self.l_handtip,
                    r_shoulder=self.r_shoulder,
                    l_shoulder=self.l_shoulder,
                    head=self.head,
                    chest=self.chest,
                    naval=self.naval,
                    is_data=self.is_data)
            self.is_hand_up, self.is_r_hand_up, self.is_l_hand_up = \
                self.act_hand_up.calculate(
                    r_wrist=self.r_wrist,
                    l_wrist=self.l_wrist,
                    r_elbow=self.r_elbow,
                    l_elbow=self.l_elbow,
                    r_handtip=self.r_handtip,
                    l_handtip=self.l_handtip,
                    r_shoulder=self.r_shoulder,
                    l_shoulder=self.l_shoulder,
                    head=self.head,
                    chest=self.chest,
                    naval=self.naval,
                    nose=self.nose,
                    is_data=self.is_data)
            self.is_hand_clap, self.is_r_hand_clap, self.is_l_hand_clap = \
                self.act_hand_clap.calculate(
                    r_wrist=self.r_wrist,
                    l_wrist=self.l_wrist,
                    r_elbow=self.r_elbow,
                    l_elbow=self.l_elbow,
                    r_handtip=self.r_handtip,
                    l_handtip=self.l_handtip,
                    r_hand=self.r_hand,
                    l_hand=self.l_hand,
                    r_shoulder=self.r_shoulder,
                    l_shoulder=self.l_shoulder,
                    head=self.head,
                    chest=self.chest,
                    naval=self.naval,
                    is_data=self.is_data)
            self.is_hand_x, self.is_hand_y, self.is_hand_z = \
                self.act_hand_stat.calculate(
                    r_wrist=self.r_wrist,
                    l_wrist=self.l_wrist,
                    r_elbow=self.r_elbow,
                    l_elbow=self.l_elbow,
                    r_handtip=self.r_handtip,
                    l_handtip=self.l_handtip,
                    r_hand=self.r_hand,
                    l_hand=self.l_hand,
                    r_shoulder=self.r_shoulder,
                    l_shoulder=self.l_shoulder,
                    head=self.head,
                    chest=self.chest,
                    naval=self.naval,
                    is_data=self.is_data)
            self.is_screen_xy, self.is_screen_x, self.is_screen_y = \
                self.act_hand_point.calculate(
                    r_wrist=self.r_wrist,
                    l_wrist=self.l_wrist,
                    r_elbow=self.r_elbow,
                    l_elbow=self.l_elbow,
                    r_handtip=self.r_handtip,
                    l_handtip=self.l_handtip,
                    r_hand=self.r_hand,
                    l_hand=self.l_hand,
                    r_shoulder=self.r_shoulder,
                    l_shoulder=self.l_shoulder,
                    head=self.head,
                    chest=self.chest,
                    naval=self.naval,
                    is_data=self.is_data)
            self.is_throwseed, self.is_r_throwseed, self.is_l_throwseed = \
                self.act_hand_throw_seed.calculate(
                    r_wrist=self.r_wrist,
                    l_wrist=self.l_wrist,
                    r_elbow=self.r_elbow,
                    l_elbow=self.l_elbow,
                    r_handtip=self.r_handtip,
                    l_handtip=self.l_handtip,
                    r_hand=self.r_hand,
                    l_hand=self.l_hand,
                    r_shoulder=self.r_shoulder,
                    l_shoulder=self.l_shoulder,
                    head=self.head,
                    chest=self.chest,
                    naval=self.naval,
                    is_data=self.is_data)
                    #---
            self.throw_shuriken, self.is_r_shuriken, self.is_l_shuriken = \
                self.act_hand_shuriken.calculate(
                    r_wrist=self.r_wrist,
                    l_wrist=self.l_wrist,
                    r_elbow=self.r_elbow,
                    l_elbow=self.l_elbow,
                    r_handtip=self.r_handtip,
                    l_handtip=self.l_handtip,
                    r_hand=self.r_hand,
                    l_hand=self.l_hand,
                    r_shoulder=self.r_shoulder,
                    l_shoulder=self.l_shoulder,
                    head=self.head,
                    chest=self.chest,
                    naval=self.naval,
                    is_data=self.is_data)
                    #---
            self.is_hand_lights, self.is_r_hand_lights, self.is_l_hand_lights = \
                self.act_hand_lights.calculate(
                    r_wrist=self.r_wrist,
                    l_wrist=self.l_wrist,
                    r_elbow=self.r_elbow,
                    l_elbow=self.l_elbow,
                    r_handtip=self.r_handtip,
                    l_handtip=self.l_handtip,
                    r_hand=self.r_hand,
                    l_hand=self.l_hand,
                    r_shoulder=self.r_shoulder,
                    l_shoulder=self.l_shoulder,
                    head=self.head,
                    chest=self.chest,
                    naval=self.naval,
                    is_data=self.is_data)
        else:
            self.is_hand_swing = 0
            self.is_r_hand_swing = 0
            self.is_l_hand_swing = 0
            self.is_hand_swipe = 0
            self.is_r_hand_swipe = 0
            self.is_l_hand_swipe = 0
            self.is_hand_push = 0
            self.is_r_hand_push = 0
            self.is_l_hand_push = 0
            self.is_hand_up = 0
            self.is_r_hand_up = 0
            self.is_l_hand_up = 0
            self.is_hand_clap = 0
            self.is_r_hand_clap = 0
            self.is_l_hand_clap = 0
            self.is_hand_x = 0
            self.is_hand_y = 0
            self.is_hand_z = 0
            self.is_screen_xy = 0
            self.is_screen_x = 0
            self.is_screen_y = 0
            self.is_throwseed = 0
            self.is_r_throwseed = 0
            self.is_l_throwseed = 0
            self.throw_shuriken = 0
            self.is_r_shuriken = 0
            self.is_l_shuriken = 0
            self.is_hand_lights = 0
            self.is_r_hand_lights= 0
            self.is_l_hand_lights = 0
        # データ格納
        self.set_data()
