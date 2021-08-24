#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import time

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Prefix.preprocessor import Preprocessor
from DetectBodyStatus.processor import Processor
from Postfix.postprocessor import Postprocessor

class Detect_rule:
    def __init__(self, axis=3):
        self.is_data = False
        self.axis = axis

        self.prefix = Preprocessor()
        self.bodystatus = Processor()
        self.postfix = Postprocessor()

    def Calculat(self, dict_data, get_time):
        self.Calculate(dict_data, get_time)
        return self.output_data

    def Calculate(self, dict_data, get_time):
        self.is_data = False
        try:
            if(dict_data['joints'] != {}):
                self.is_data = True
        except Exception as e:
            self.is_data = False

        self.prefix.Calculate(dict_data, is_data=self.is_data)
        body_dict = self.prefix.output_data

        self.bodystatus.Calculate(body_dict['smooth'], is_data=self.is_data)
        status_dict = self.bodystatus.output_data

        self.postfix.Calculate(status_dict, is_data=self.is_data)
        status_dict = self.postfix.output_data
        data = {}
        data['body'] = body_dict
        data['status'] = status_dict
        self.dict_result = data
