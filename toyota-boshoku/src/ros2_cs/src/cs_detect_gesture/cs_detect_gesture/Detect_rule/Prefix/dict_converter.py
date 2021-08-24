#!/usr/bin/env python
# 20200415 fit SDK v 1.0.1 by TM kameda
# 20200625 add get confidence data and raw data by TM kameda
# 20200703 add is_data_error by TM kameda

from collections import OrderedDict

class DictConverter:
    def __init__(self,
                 axis=3,
                 joint=31,
                 ):
        self.axis = axis
        self.axis_d = ['x', 'y', 'z']
        self.joint = joint
        self.convert_list = [
            ['pelvis', 0],
            ['naval', 1],
            ['chest', 2],
            ['neck', 3],
            ['l_clavicle', 4],
            ['l_shoulder', 5],
            ['l_elbow', 6],
            ['l_wrist', 7],
            ['l_hand', 8],
            ['l_handtip', 9],
            ['l_thumb', 10],
            ['r_clavicle', 11],
            ['r_shoulder', 12],
            ['r_elbow', 13],
            ['r_wrist', 14],
            ['r_hand', 15],
            ['r_handtip', 16],
            ['r_thumb', 17],
            ['l_hip', 18],
            ['l_knee', 19],
            ['l_ankle', 20],
            ['l_foot', 21],
            ['r_hip', 22],
            ['r_knee', 23],
            ['r_ankle', 24],
            ['r_foot', 25],
            ['head', 26],
            ['nose', 27],
            ['l_eyes', 28],
            ['l_ear', 29],
            ['r_eyes', 30],
            ['r_ear', 31],
        ]
        self.converted_dict = OrderedDict()
        self.nodata = -99

    def ConvertDict(self, dic_part):
        o_dict = OrderedDict()
        o_dict['raw'] = OrderedDict()
        o_dict['smooth'] = OrderedDict()
        if not (dic_part is None):
            for i in range(len(self.convert_list)):
                convert_box = self.convert_list[i]
                key = convert_box[0]
                ikey = convert_box[1]
                p_dict = OrderedDict()
                r_dict = OrderedDict()
                for ax in range(self.axis):
                    p_dict[self.axis_d[ax]] = dic_part['smooth']['joints'][ikey]['position'][ax]
                    r_dict[self.axis_d[ax]] = dic_part['raw']['joints'][ikey]['position'][ax]
                p_dict['confidence'] = dic_part['smooth']['joints'][ikey].get(
                    'confidence')
                r_dict['confidence'] = dic_part['raw']['joints'][ikey].get(
                    'confidence')
                #o_dict[key] = p_dict
                o_dict['smooth'][key] = p_dict
                o_dict['raw'][key] = r_dict
            o_dict['body_id'] = dic_part['raw']['id']
            o_dict['raw_camera_ids'] = dic_part['raw'].get('raw_camera_ids')
            o_dict['is_data'] = True
            is_error = dic_part.get('is_body_error', False)
            o_dict['is_body_error'] = bool(is_error)
        else:
            for i in range(len(self.convert_list)):
                convert_box = self.convert_list[i]
                key = convert_box[0]
                ikey = convert_box[1]
                p_dict = OrderedDict()
                r_dict = OrderedDict()
                for ax in range(self.axis):
                    p_dict[self.axis_d[ax]] = 0
                    r_dict[self.axis_d[ax]] = 0
                p_dict['confidence'] = 0
                r_dict['confidence'] = 0
                #o_dict[key] = p_dict
                o_dict['smooth'][key] = p_dict
                o_dict['raw'][key] = r_dict
            o_dict['body_id'] = self.nodata
            o_dict['raw_camera_ids'] = {}
            o_dict['is_data'] = False
            o_dict['is_body_error'] = False
        self.converted_dict = o_dict
