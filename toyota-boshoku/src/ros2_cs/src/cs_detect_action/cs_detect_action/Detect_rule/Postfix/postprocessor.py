#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

class Postprocessor:
    # 後処理によるUX向上処理
    def __init__(self, axis=3):
        self.axis = axis
        self.is_data = False
        self.output_data = {}

    def Calculate(self, input_dict, is_data=False):
        self.output_data = input_dict
