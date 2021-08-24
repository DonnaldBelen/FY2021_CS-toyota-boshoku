# coding:utf-8
from collections import deque
import numpy as np
import math
import os
import configparser

class Act_Hand_Swipe:
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

        self.delay = inifile.getint('gesture_recognition','release')
        self.movement_threshold = inifile.getint('gesture_recognition','movement_threshold')
        self.swipe_boundary = inifile.getint('gesture_recognition','swipe_boundary')

        self.is_r_hand_swipe = 0
        self.is_l_hand_swipe = 0
        self.is_hand_swipe = 0
        self.prev_r_hand = np.zeros((axis))
        self.prev_l_hand = np.zeros((axis))
        self.release = 0

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
        self.is_hand_swipe = 0
        self.is_l_hand_swipe = 0
        self.is_r_hand_swipe = 0

        x_idx = 0
        y_idx = 1
        z_idx = 2

        delay = self.delay
        thresh_size = self.movement_threshold

        if (is_data):
            if self.is_base_axis(r_shoulder,r_elbow,r_handtip,naval,r_elbow,r_wrist) or self.is_base_axis(r_shoulder,r_elbow,r_handtip,naval,l_elbow,l_wrist):
                self.release -= 1
                if r_hand[y_idx] > chest[y_idx] and r_hand[y_idx] < head[y_idx] and l_hand[y_idx] > chest[y_idx] and l_hand[y_idx] < head[y_idx]:
                    if self.prev_r_hand[x_idx] != 0  and self.release < 0:
                        if abs(self.prev_r_hand[x_idx] - r_hand[x_idx]) > thresh_size and abs(self.prev_r_hand[y_idx] - r_hand[y_idx]) < thresh_size:
                            self.is_r_hand_swipe = 1
                            direction = self.prev_r_hand[x_idx] - r_hand[x_idx]
                            if direction > 0:
                                self.is_hand_swipe = 2
                                self.release = delay
                            else:
                                self.is_hand_swipe = 1
                                self.release = delay
                    self.prev_r_hand[x_idx] = r_hand[x_idx]
                    self.prev_r_hand[y_idx] = r_hand[y_idx]

                    if self.prev_l_hand[x_idx] != 0 and self.release < 0:
                        if abs(self.prev_l_hand[x_idx] - l_hand[x_idx]) > thresh_size and abs(self.prev_l_hand[y_idx] - l_hand[y_idx]) < thresh_size:
                            self.is_l_hand_swipe = 1
                            direction = self.prev_l_hand[x_idx] - l_hand[x_idx]
                            if direction > 0:
                                self.is_hand_swipe = 2
                                self.release = delay
                            else:
                                self.is_hand_swipe = 1
                                self.release = delay
                    self.prev_l_hand[x_idx] = l_hand[x_idx]
                    self.prev_l_hand[y_idx] = l_hand[y_idx]

        #self.is_hand_swipe = abs(self.prev_r_hand - r_hand[x_idx])
        return self.is_hand_swipe, self.is_r_hand_swipe, self.is_l_hand_swipe

    def is_base_axis(self, shoulder, elbow, handtip, naval, rl_elbow, rl_wrist):
        x_idx = 0
        y_idx = 1
        z_idx = 2

        base_check = 0

        if rl_wrist[y_idx] > naval[y_idx]:
            base_check = 1
            return base_check
        return base_check
