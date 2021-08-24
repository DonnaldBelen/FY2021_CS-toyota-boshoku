#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import time

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Prefix.preprocessor import Preprocessor
from DetectBodyStatus.processor import Processor
from DetectBodyProperty.property_processor import PropertyProcessor
from Postfix.postprocessor import Postprocessor

class Detect_rule:
    def __init__(self, axis=3, ids='1'):
        self.is_data = False
        self.axis = axis

        self.prefix = Preprocessor()
        self.bodystatus = Processor(ids=ids)
        self.bodystatus2 = Processor(ids=ids)
        self.bodyproperty = PropertyProcessor()
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
        self.bodystatus2.Calculate(body_dict['raw'], is_data=self.is_data)
        status_dict2 = self.bodystatus2.output_data

        self.bodyproperty.Calculate(body_dict['smooth'], is_data=self.is_data)
        property_dict = self.bodyproperty.output_data

        self.postfix.Calculate(status_dict, is_data=self.is_data)
        status_dict = self.postfix.output_data
        data = {}
        data['body'] = body_dict
        data['status'] = status_dict
        data['property'] = property_dict['property']
        data['fast_status'] = status_dict2
        data['detect_action_version'] = 20201105
        self.dict_result = data
