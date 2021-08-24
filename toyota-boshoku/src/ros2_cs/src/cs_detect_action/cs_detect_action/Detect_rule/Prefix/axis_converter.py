#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 更新履歴
# 20191120 type 71 をtype 78軸方向へ変換するアダプターとして開発
# 変換軸要素、軸向きの変換アダプター
# 20191122 type78 to type 71 AzureKinect→統合座標系への変換
# convert_axis_a=[0, 1, 2],
# convert_axis_c=[-1, -1, 1],
# 20200410 type78 を基準に変更 将来の変更に備え変換機能は保持
# convert_axis_a=[0, 1, 2],
# convert_axis_c=[1, 1, 1],


class AxisConverter:
    def __init__(self,
                 axis=3,
                 joint=31,
                 convert_axis_a=[0, 1, 2],
                 convert_axis_c=[1, 1, 1],
                 ):
        self.axis = axis
        self.joint = joint
        self.ax_a = convert_axis_a
        self.ax_c = convert_axis_c
        self.converted_dict = {}

    def ConvertAxis(self, dic_data):
        # 軸変換 type 78 to type 71(AzKinect axis)
        # 'x','z','y'
        ax_a = self.ax_a
        ax_c = self.ax_c
        if not (dic_data is None):
            c_dic = dic_data.copy()
            for jo in range(self.joint):
                for ax in range(self.axis):
                    point = dic_data['joints'][jo]['position'][ax_a[ax]]
                    point = point * ax_c[ax]
                    c_dic['joints'][jo]['position'][ax] = point
            self.converted_dict = c_dic
        else:
            self.converted_dict = dic_data
