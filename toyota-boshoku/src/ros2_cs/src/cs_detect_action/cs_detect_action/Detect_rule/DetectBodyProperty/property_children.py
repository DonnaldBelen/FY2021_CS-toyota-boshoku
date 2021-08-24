# coding:utf-8
import numpy as np

class Property_children:
    def __init__(self, threshold=150):
        # 読み込み用軸パラメータ
        self.axis = axis = 3
        # 計算入力
        self.is_children = False
        self.height = 0.0
        self.threshold = threshold

    def calculate(self,
                  height=0.0,
                  is_data=False):
        # データある時の出力
        if (is_data):
            if(height < self.threshold):
                self.is_children = True
            else:
                self.is_children = False
        return self.is_children
