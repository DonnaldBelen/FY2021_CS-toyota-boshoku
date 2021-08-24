#!/usr/bin/env python

class Axis_Converter:
    def __init__(self,
                 axis=3,
                 joint=31,
                 convert_axis_a=[0, 1, 2],
                 convert_axis_c=[-1, -1, 1],
                 ):
        self.axis = axis
        self.joint = joint
        self.ax_a = convert_axis_a
        self.ax_c = convert_axis_c

    def Convert_Axis(self, dic_data):
        self.c_dic = dic_data
        return self.c_dic
