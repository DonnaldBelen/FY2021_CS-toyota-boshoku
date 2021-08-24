# coding:utf-8
from collections import deque
import numpy as np

class Property_hight:
    def __init__(self, window_size=20):
        # 読み込み用軸パラメータ
        self.axis = axis = 3
        # 計算入力
        self.l_shoulder = np.zeros((axis))
        self.r_shoulder = np.zeros((axis))
        self.l_elbow = np.zeros((axis))
        self.r_elbow = np.zeros((axis))
        self.l_hand = np.zeros((axis))
        self.r_hand = np.zeros((axis))
        self.l_wrist = np.zeros((axis))
        self.r_wrist = np.zeros((axis))
        self.l_handtip = np.zeros((axis))
        self.r_handtip = np.zeros((axis))
        self.height = 0.0
        self.data_tank = deque(maxlen=window_size)

    def calculate(self,
                  l_shoulder=np.zeros(3),
                  r_shoulder=np.zeros(3),
                  l_elbow=np.zeros(3),
                  r_elbow=np.zeros(3),
                  l_hand=np.zeros(3),
                  r_hand=np.zeros(3),
                  l_wrist=np.zeros(3),
                  r_wrist=np.zeros(3),
                  l_handtip=np.zeros(3),
                  r_handtip=np.zeros(3),
                  is_data=False):
        # 初期値
        self.height = 0.0
        # データある時の出力
        if (is_data):
            #box = [r_handtip, r_wrist, r_hand, r_elbow,
            #       r_shoulder, l_shoulder, 
            #       l_elbow, l_hand, l_wrist, l_handtip,
            #       ]
            box = [r_hand, r_wrist, r_hand, r_elbow,
                   r_shoulder, l_shoulder,
                   l_elbow, l_hand, l_wrist, l_hand,
                   ]
            S = 0.0
            for i in range(len(box)-1):
                d = box[i]-box[i+1]
                S += np.linalg.norm(d, ord=2)
            height = S/10.0
            self.data_tank.append(height)
            self.height = np.nanmedian(self.data_tank)
        return self.height
