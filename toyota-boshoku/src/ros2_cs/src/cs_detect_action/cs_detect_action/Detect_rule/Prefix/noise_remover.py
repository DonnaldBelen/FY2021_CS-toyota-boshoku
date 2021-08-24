#!/usr/bin/env python
# -*- coding: utf-8 -*-
from collections import deque
import numpy as np
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Prefix.body_checker import BodyChecker

class NoiseRemover:
    def __init__(self,
                 axis=3,
                 joint=31,
                 window_size=5,
                 ):
        self.axis = axis
        self.joint = joint

        self.data_tank =[]
        for i in range(joint):
            box=[]
            for j in range(axis):
                box.append(deque(maxlen=window_size))
            self.data_tank.append(box)

        self.dic_removed_noise = {}
        self.is_body_error = False
        self.bodychecker = BodyChecker()

    def Remove_Noise(self, dic_data):
        if not (dic_data is None):
            self.is_body_error = self.bodychecker.CheckError(dic_data)
            c_dic = {}
            c_dic['smooth'] = {}
            c_dic['smooth'] = dic_data.copy()
            #c_dic['smooth']['is_body_error'] = self.is_body_error
            c_dic['raw'] = {}
            c_dic['raw'] = dic_data.copy()
            #c_dic['raw']['is_body_error'] = self.is_body_error

            for jo in range(self.joint):
                for ax in range(self.axis):
                    point = dic_data['joints'][jo]['position'][ax]
                    self.data_tank[jo][ax].append(point)
                    point = np.nanmedian(self.data_tank[jo][ax])
                    c_dic['smooth']['joints'][jo]['position'][ax] = point

            self.dic_removed_noise = c_dic
            self.dic_removed_noise['is_body_error'] = self.is_body_error
        else:
            self.dic_removed_noise = dic_data
            self.is_body_error = False
        return self.dic_removed_noise
