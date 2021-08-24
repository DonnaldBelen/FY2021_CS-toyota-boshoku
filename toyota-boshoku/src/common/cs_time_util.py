# -*- coding: utf-8 -*-
import ntplib
import configparser
import os
import datetime
from time import ctime

class CSTimeUtil(object):
    def __init__(self):
        # 設定読み込み
        inifile = configparser.ConfigParser()
        inifile_dir = os.path.dirname(os.path.abspath(__file__))
        inifile_path = "%s/config.ini" % inifile_dir
        for i in range(10):
            if os.path.exists(inifile_path):
                break
            else:
                inifile_dir = "%s/.." % inifile_dir
                inifile_path = "%s/config.ini" % inifile_dir
        print("load %s" % inifile_path)
        inifile.read(inifile_path, 'UTF-8')

        self.__ntp_server_host = inifile.get('common', 'ntp_server_host')
        self.__ntp = ntplib.NTPClient()
        
    def get_time(self):
        try:
            return self.__ntp.request(self.__ntp_server_host).tx_time
        except ntplib.NTPException:
            return -1

    def get_datetime(self, tx_time = -1):

        if tx_time < 0:
            tx_time = self.get_time()

        time_str = ctime(tx_time)
        time = datetime.datetime.strptime(time_str, "%a %b %d %H:%M:%S %Y")

        microseconds = (tx_time - int(tx_time)) * 1000 * 1000
        time = time + datetime.timedelta(microseconds=microseconds)

        return time

    def get_sensing_time(self, tx_time = -1):
        return self.get_datetime(tx_time).strftime("%Y-%m-%d %H:%M:%S.%f")

def test():
    time_util = CSTimeUtil()
    print(time_util.get_time())
    print(time_util.get_datetime())
    print(time_util.get_sensing_time())

if __name__ == '__main__':
    test()