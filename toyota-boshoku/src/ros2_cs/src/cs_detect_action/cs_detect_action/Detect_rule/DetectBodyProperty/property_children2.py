# coding:utf-8
from collections import deque
import numpy as np

class Property_children2:
    def __init__(self, threshold=1.3, window_size=20):
        # 肩幅と頭部幅の比による判定　子供は1.0+腕の太さ、大人は肩幅が広くなる 
        # 読み込み用軸パラメータ
        self.axis = axis = 3
        # 計算入力
        self.is_children = False
        self.ratio = 0.0
        self.threshold = threshold
        self.data_tank = deque(maxlen=window_size)

    def calculate(self,
                  l_ear=np.zeros(3),
                  r_ear=np.zeros(3),
                  l_shoulder=np.zeros(3),
                  r_shoulder=np.zeros(3),
                  is_data=False):
        # データある時の出力
        if (is_data):
            head_L = np.linalg.norm((l_ear - r_ear), ord=2)
            shoulder_L = np.linalg.norm((l_shoulder - r_shoulder), ord=2)
            ratio = shoulder_L / head_L
            self.data_tank.append(ratio)
            ratio = np.nanmedian(self.data_tank)
            self.ratio = ratio
            if(self.ratio < self.threshold):
                self.is_children = True
            else:
                self.is_children = False
        return self.is_children
