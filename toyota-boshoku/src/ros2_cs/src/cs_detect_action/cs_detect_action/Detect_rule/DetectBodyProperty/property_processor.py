#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from detect_property import Detect_property

class PropertyProcessor:
    def __init__(self):
        self.body_property = Detect_property()

    def Calculate(self, data_dict, is_data=False):
        self.body_property.Calculate(data_dict, is_data=is_data)
        body_property_dict = self.body_property.output_data

        output_dict = {}
        output_dict['property'] = body_property_dict
        self.output_data = output_dict
