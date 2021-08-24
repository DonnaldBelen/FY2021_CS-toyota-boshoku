# coding:utf-8
from collections import deque
import numpy as np
import math
import os
import configparser

class Act_Hand_Point:
    def __init__(self, threshold=20, window_size=5):
        # 肘基準で拳が動き続ける状態 1.0s平均で、しきい値以上の運動をしているか？
        # 読み込み用軸パラメータ
        self.axis = axis = 3
        self.window_size = window_size
        # しきい値^
        self.threshold = threshold

        # 設定読み込み
        inifile = configparser.ConfigParser()
        inifile.read(os.path.dirname(os.path.abspath(__file__)) + '/../../../../../../config.ini', 'UTF-8')
        self.Screen_z = inifile.getint('gesture_recognition','Screen_z')
        self.Point_boundary = inifile.getint('gesture_recognition','Point_boundary')

        # 計算入力
        self.r_wrist = np.zeros((axis))
        self.l_wrist = np.zeros((axis))
        self.r_elbow = np.zeros((axis))
        self.l_elbow = np.zeros((axis))
        self.r_handtip = np.zeros((axis))
        self.l_handtip = np.zeros((axis))
        self.r_hand = np.zeros((axis))
        self.l_hand = np.zeros((axis))
        self.r_thumb = np.zeros((axis))
        self.l_thumb = np.zeros((axis))
        self.head = np.zeros((axis))
        self.chest = np.zeros((axis))
        self.naval = np.zeros((axis))
        self.nose = np.zeros((axis))

        self.is_hand_xy = 0
        self.is_r_hand_xy = 0
        self.is_l_hand_xy = 0

        self.vertex_x1 = [514, 110, -294, -698]
        self.vertex_y1 = [847, 619, 391, 163]

    def calculate(self,
                  r_wrist=np.zeros(3),
                  l_wrist=np.zeros(3),
                  r_elbow=np.zeros(3),
                  l_elbow=np.zeros(3),
                  r_handtip=np.zeros(3),
                  l_handtip=np.zeros(3),
                  r_hand=np.zeros(3),
                  l_hand=np.zeros(3),
                  r_shoulder=np.zeros(3),
                  l_shoulder=np.zeros(3),
                  head=np.zeros(3),
                  chest=np.zeros(3),
                  naval=np.zeros(3),
                  nose=np.zeros(3),
                  is_data=False):
        # 初期値
        self.is_hand_xy = 0
        self.is_r_hand_xy = 0
        self.is_l_hand_xy = 0
        Screen_z = self.Screen_z
        Point_boundary = self.Point_boundary
        x_idx = 0
        y_idx = 1
        z_idx = 2

        if (is_data):
            self.is_hand_xy = 0
            self.is_r_hand_xy = 0
            self.is_l_hand_xy = 0
            if r_hand[y_idx] > naval[y_idx]:
                if r_hand[z_idx] > -Point_boundary:
                    hand = r_hand
                    elbow = r_elbow
                    self.is_r_hand_xy = 1
            if l_hand[y_idx] > naval[y_idx]:
                if l_hand[z_idx] > -Point_boundary:
                    hand = l_hand
                    elbow = l_elbow
                    self.is_l_hand_xy = 1
            if self.is_r_hand_xy == 1 and self.is_l_hand_xy == 1:
                    hand = r_hand
                    elbow = r_elbow
            if self.is_r_hand_xy == 1 or self.is_l_hand_xy == 1:
                Screen = np.zeros((self.axis))
                Screen[z_idx] = Screen_z
                hand[x_idx] = hand[x_idx] - elbow[x_idx]
                hand[y_idx] = hand[y_idx] - elbow[y_idx]
                hand[z_idx] = hand[z_idx] - elbow[z_idx]
                if hand[z_idx] > elbow[z_idx]:
                    Screen[x_idx] =  hand[x_idx] * ( Screen[z_idx]/ hand[z_idx])
                    Screen[y_idx] =  hand[y_idx] * ( Screen[z_idx]/ hand[z_idx])
                    for idx in range(0,2):
                        Screen[idx] = Screen[idx] + elbow[idx]
                self.is_hand_xy = self.get_Pos(Screen[x_idx], Screen[y_idx])

                return self.is_hand_xy, Screen[x_idx] , Screen[y_idx]
            return 0, 0, 0

    def get_Pos(self, x_comp, y_comp):
        cp_x = x_comp
        cp_y = y_comp

        result = 0

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
