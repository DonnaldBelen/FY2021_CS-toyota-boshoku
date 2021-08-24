# coding:utf-8
from collections import deque
import numpy as np
import math
import os
import configparser

class Act_Hand_Up:
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
        self.hand_ht = inifile.getint('gesture_recognition','hand_ht')

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

        self.is_r_hand_up = 0
        self.is_l_hand_up = 0
        self.is_hand_up = 0

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
        self.is_r_hand_up = 0
        self.is_l_hand_up = 0
        self.is_hand_up = 0
        hand_ht = self.hand_ht

        x_idx = 0
        y_idx = 1
        z_idx = 2

        if (is_data):
            self.is_hand_up = 0
            self.is_r_hand_up = 0
            self.is_l_hand_up = 0
            if l_wrist[y_idx] > (nose[y_idx] + hand_ht):
                self.is_l_hand_up = 1
            if r_wrist[y_idx] > (nose[y_idx] + hand_ht):
                self.is_r_hand_up = 1
            if self.is_r_hand_up or self.is_l_hand_up:
                self.is_hand_up = 1

            up_val = []
            up_val.append(self.is_l_hand_up)
            up_val.append(self.is_r_hand_up)
            self.is_hand_up = max(up_val)

        return self.is_hand_up, self.is_r_hand_up, self.is_l_hand_up
