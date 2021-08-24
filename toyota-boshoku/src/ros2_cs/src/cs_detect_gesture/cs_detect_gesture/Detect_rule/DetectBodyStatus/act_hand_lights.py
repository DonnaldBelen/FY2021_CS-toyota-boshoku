# coding:utf-8
from collections import deque
import numpy as np
import math
import os
import configparser


class Act_Hand_Lights:
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
        self.thresh_small = inifile.getint('gesture_recognition','thresh_wave_small')
        self.thresh_med = inifile.getint('gesture_recognition','thresh_wave_medium')
        self.thresh_large = inifile.getint('gesture_recognition','thresh_wave_large')
        deque_size = inifile.getint('gesture_recognition','deque_size')
        self.swing_max_dif = inifile.getint('gesture_recognition','swing_max_dif')
        self.swing_min_dif = inifile.getint('gesture_recognition','swing_min_dif')
        self.hand_offset = inifile.getint('gesture_recognition','hand_offset')

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



        self.handtip_L_x_recent = deque([0],maxlen=deque_size)
        self.handtip_R_x_recent = deque([0],maxlen=deque_size)
        self.window_move_hilo_R = deque([0],maxlen=deque_size)
        self.window_move_hilo_L = deque([0],maxlen=deque_size)

        self.is_r_hand_lights = 0
        self.is_l_hand_lights = 0
        self.is_hand_lights = 0


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
        self.is_hand_lights = 0
        self.is_r_hand_lights = 0
        self.is_l_hand_lights = 0

        is_r_hand_up = False
        is_l_hand_up = False
        is_hand_up = False

        x_idx = 0
        y_idx = 1
        z_idx = 2

        thresh_small = self.thresh_small
        thresh_med = self.thresh_med
        thresh_large = self.thresh_large

        l_handtip_dif = 0
        r_handtip_dif = 0

        flag_movebothways_L = False
        flag_movebothways_R = False

        if (is_data):
            if not(r_hand[y_idx] > chest[y_idx] and r_hand[y_idx] < head[y_idx] and l_hand[y_idx] > chest[y_idx] and l_hand[y_idx] < head[y_idx]):
                l_handtip_dif = l_handtip[x_idx] -self.handtip_L_x_recent[-1]
                r_handtip_dif = r_handtip[x_idx] -self.handtip_R_x_recent[-1]
                if abs(l_handtip_dif) < self.swing_max_dif:
                    self.handtip_L_x_recent.append(l_handtip[x_idx])
                    if l_handtip_dif > self.swing_min_dif:
                        self.window_move_hilo_L.append(1)
                    elif l_handtip_dif < -self.swing_min_dif:
                        self.window_move_hilo_L.append(-1)
                    else:
                        self.window_move_hilo_L.append(0)
                if abs(r_handtip_dif) < self.swing_max_dif:
                    self.handtip_R_x_recent.append(r_handtip[x_idx])
                    if r_handtip_dif > self.swing_min_dif:
                        self.window_move_hilo_R.append(1)
                    elif r_handtip_dif < -self.swing_min_dif:
                        self.window_move_hilo_R.append(-1)
                    else:
                        self.window_move_hilo_R.append(0)

                if max(self.window_move_hilo_L)- min(self.window_move_hilo_L) == 2:
                    flag_movebothways_L = True
                if max(self.window_move_hilo_R)- min(self.window_move_hilo_R) == 2:
                    flag_movebothways_R = True

                self.is_hand_lights = 0
                self.is_l_hand_lights = 0
                self.is_r_hand_lights = 0

                move_amnt_R = np.percentile(self.handtip_R_x_recent,90) - np.percentile(self.handtip_R_x_recent,10)
                move_amnt_L = np.percentile(self.handtip_L_x_recent,90) - np.percentile(self.handtip_L_x_recent,10)

                if flag_movebothways_R:
                    if (move_amnt_R) > thresh_small:
                        if r_handtip[y_idx] > (r_hand[y_idx]+self.hand_offset):
                            if r_wrist[y_idx] > r_elbow[y_idx]:
                                if (r_wrist[y_idx] > naval[y_idx]):
                                    is_r_hand_up = True
                                    self.is_r_hand_lights = 1
                                    if (move_amnt_R) > thresh_large:
                                        self.is_r_hand_lights = 3
                                    elif (move_amnt_R) > thresh_med:
                                        self.is_r_hand_lights = 2
                if flag_movebothways_L:
                    if (move_amnt_L) > thresh_small:
                        if l_handtip[y_idx] > (l_hand[y_idx]+self.hand_offset):
                            if l_wrist[y_idx] > l_elbow[y_idx]:
                                if (l_wrist[y_idx] > naval[y_idx]):
                                    is_l_hand_up = True
                                    self.is_l_hand_lights = 1
                                    if (move_amnt_L) > thresh_large:
                                        self.is_l_hand_lights = 3
                                    elif (move_amnt_L) > thresh_med:
                                        self.is_l_hand_lights = 2

            swing_val = []
            swing_val.append(self.is_r_hand_lights)
            swing_val.append(self.is_l_hand_lights)
            self.is_hand_lights = max(swing_val)

        return self.is_hand_lights, self.is_r_hand_lights, self.is_l_hand_lights
