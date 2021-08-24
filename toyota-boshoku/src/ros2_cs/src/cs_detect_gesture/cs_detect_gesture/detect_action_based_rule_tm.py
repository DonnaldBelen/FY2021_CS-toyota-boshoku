#!/usr/bin/env python
# coding: utf-8

import configparser
import json
import os
import sys
import time

# 設定読み込み用
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../../predict/')
from predict_json_getter import MyEncoder, PredictJsonGetter

# 前処理用
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Detect_rule.detect_rule import Detect_rule
from Preprocessing.Preprocessor import Preprocessor

class DetectActionBasedRule():
    def __init__(self):
        inifile = configparser.ConfigParser()
        inifile.read(
            os.path.dirname(
                os.path.abspath(__file__)) +
            '/../../config.ini',
            'UTF-8')

        self.upload_dir = '{}/{}'.format(inifile.get('common', 'work_dir'),
                                    inifile.get('common', 'upload_dir'))
        os.makedirs(self.upload_dir, exist_ok=True)

        # input
        self.json_type = '78'
        self.json_no = '1'
        self.char_ids = ['1', '2', '3']
        # output
        self.out_json_type = '75'
        self.set_wait = 0.1

        # 入力用
        self.predict_json_getter = PredictJsonGetter()

        # 前処理 被験者間で行う前処理
        self.prep = Preprocessor()

        self.detect_actions = []
        for i in range(len(self.char_ids)):
            self.detect_actions.append(Detect_rule())


    def save_upload_data(self, data, sensor_type, sensor_no, sensing_time):
        frame_data = {
            'sensing_time': sensing_time,
            'type': sensor_type,
            'no': sensor_no,
            'data': data
        }

        save_json_file = '{}/{}_{}_{}'.format(self.upload_dir,
                                            sensing_time, sensor_type, sensor_no)
        with open(save_json_file, 'w') as f:
            json.dump(frame_data, f, cls=MyEncoder)
            f.close()
            os.rename(save_json_file, '{}.json'.format(save_json_file))

    def run(self):
        # start recognition
        start_time = time.time()
        self.get_logger().info('start:', start_time)

        while(True):
            tic = time.time()

            json_data = self.predict_json_getter.get_from_db(
                window_size=1, type_no={self.json_type: self.json_no})
            if len(json_data[self.json_type][self.json_no]) == 0:
                self.get_logger().error('ERROR : could not get predict_json')
                continue
            sensing_time = json_data[self.json_type][self.json_no][0][3]
            Az_data = json_data[self.json_type][self.json_no][0][5]
            dic_data = json.loads(Az_data)

            # 動作確認用
            # out_box = []
            for i, char_id in enumerate(self.char_ids):
                _key = 'seat' + char_id

                input_dict = self.prep.Calculate(dic_data[_key])
                self.detect_actions[i].Calculate(input_dict, sensing_time)
                dict_result = self.detect_actions[i].dict_result
                # 動作確認用
                self.get_logger().info("%s" % dict_result)

                self.save_upload_data(
                    dict_result, self.out_json_type, char_id, sensing_time)

            #     # 動作確認用
            #     mess = 'Seat_ID: {} face_ward_relative:{:.2f}'.format(
            #         char_id, dict_result['status']['face_direction']['face_direction_horizontal'])
            #     out_box.append(mess)
            # self.get_logger().info("%s" % out_box)

            wait_time = self.set_wait - (time.time() - tic)
            if(wait_time < 0):
                wait_time = 0.0
            time.sleep(wait_time)


def main():
    do = DetectActionBasedRule()
    do.run()

if __name__ == '__main__':
    main()
