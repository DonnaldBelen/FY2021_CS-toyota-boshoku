# coding:utf-8
from collections import deque
import numpy as np
import math
import os
import configparser

class Act_Hand_Throw_Seed:
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

        deque_z_size = inifile.getint('gesture_recognition','seed_deque_size')
        deque_timecooldown_size = inifile.getint('gesture_recognition','seed_cooldown')
        self.seed_naval_z_dist = inifile.getint('gesture_recognition','seed_naval_z_dist')
        self.seed_motion_thresh = inifile.getint('gesture_recognition','seed_motion_thresh')
        self.hand_L_z_recent = deque([0],maxlen=deque_z_size)
        self.hand_R_z_recent = deque([0],maxlen=deque_z_size)
        self.time_cooldown_l = deque([0],maxlen=deque_timecooldown_size)
        self.time_cooldown_r = deque([0],maxlen=deque_timecooldown_size)

        self.is_r_throwseed = 0
        self.is_l_throwseed = 0
        self.is_throwseed = 0

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
        self.is_r_throwseed = 0
        self.is_l_throwseed = 0
        self.is_throwseed = 0

        x_idx = 0
        y_idx = 1
        z_idx = 2

        if (is_data):
            self.hand_L_z_recent.append(l_hand[z_idx])
            self.hand_R_z_recent.append(r_hand[z_idx])
            check_z_motion_L = (self.hand_L_z_recent[-1]) - (self.hand_L_z_recent[0])
            check_z_motion_R = (self.hand_R_z_recent[-1]) - (self.hand_R_z_recent[0])


            #LEFT
            if l_hand[y_idx] > naval[y_idx]:
                if l_hand[z_idx] > naval[z_idx] + self.seed_naval_z_dist:
                    if check_z_motion_L > self.seed_motion_thresh:
                        if max(self.time_cooldown_l) < 1:
                            self.is_l_throwseed =1
                            self.time_cooldown_l.append(1)
            #RIGHT
            if r_hand[y_idx] > naval[y_idx]:
                if r_hand[z_idx] > naval[z_idx] + self.seed_naval_z_dist:
                    if check_z_motion_R > self.seed_motion_thresh:
                        if max(self.time_cooldown_r) < 1:
                            self.is_r_throwseed =1
                            self.time_cooldown_r.append(1)


            if self.is_r_throwseed == 0:
                self.time_cooldown_r.append(0)
            if self.is_l_throwseed == 0:
                self.time_cooldown_l.append(0)

            up_val = []
            up_val.append(self.is_l_throwseed)
            up_val.append(self.is_r_throwseed)
            self.is_throwseed = max(up_val)


            return self.is_throwseed, self.is_r_throwseed, self.is_l_throwseed
