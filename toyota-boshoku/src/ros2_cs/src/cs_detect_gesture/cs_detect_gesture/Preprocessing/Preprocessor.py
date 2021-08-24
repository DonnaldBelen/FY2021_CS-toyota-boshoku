#!/usr/bin/env python
# -*- coding: utf-8 -*-

class Preprocessor:
    def __init__(self, axis=3, offset=0.0):
        # set parameter
        self.axis = axis
        self.is_data = False
        self.offset = offset

    def Calculate(self, dic_data):
        return dic_data
