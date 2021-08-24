#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Prefix.axis_converter import AxisConverter
from Prefix.noise_remover import NoiseRemover
from Prefix.dict_converter import DictConverter

# 更新履歴
# 2020/04/16 新規作成 by TM kameda
# 2020/06/15 パラメータ調整　NoiseRemover(joint=31, window_size=5,)　
# －＞NoiseRemover(joint=31, window_size=20,)

class Preprocessor:
    def __init__(self, axis=3):
        self.axis = axis
        self.is_data = False

        self.ConvertAxis = AxisConverter(joint=31,
                                        convert_axis_a=[0, 1, 2],
                                        convert_axis_c=[1, 1, 1],)
        self.RemoveNoise = NoiseRemover(joint=31, window_size=20,)
        self.ConvertDict = DictConverter()

        self.output_data = {}

    def Calculate(self, dic_data, is_data=False):
        self.ConvertAxis.ConvertAxis(dic_data)
        dict_converted_axis = self.ConvertAxis.converted_dict

        self.RemoveNoise.Remove_Noise(dict_converted_axis)
        dict_removed_noise = self.RemoveNoise.dic_removed_noise

        self.ConvertDict.ConvertDict(dict_removed_noise)
        dict_output = self.ConvertDict.converted_dict

        self.output_data = dict_output
