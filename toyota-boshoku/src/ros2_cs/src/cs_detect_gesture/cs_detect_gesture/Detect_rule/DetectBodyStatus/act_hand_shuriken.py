# coding:utf-8

# License
# Pyrr is released under the BSD 2-clause license (a very relaxed licence), but it is encouraged that any modifications are submitted back to the master for inclusion.
# Created by Adam Griffiths.
# Copyright (c) 2012, Twisted Pair Development. All rights reserved.
# twistedpairdevelopment.wordpress.com @twistedpairdev
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
# Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
# Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided with the distribution.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
#  BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
# IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted 
# as representing official policies, either expressed or implied, of the FreeBSD Project.


from collections import deque
import numpy as np
import math
import os
import configparser
import pyrr as py

class Act_Hand_Shuriken:
    def __init__(self, threshold=20, window_size=5):
        # 肘基準で拳が動き続ける状態 1.0s平均で、しきい値以上の運動をしているか？
        # 読み込み用軸パラメータ
        self.axis = axis = 3
        self.window_size = window_size
        # しきい値^
        self.threshold = threshold

        # 設定読み込み
        inifile = configparser.ConfigParser()
        inifile.read(os.path.dirname(os.path.abspath(__file__)) + '/../../../../../../config.ini', 'UTF-8')
        self.hand_ht = inifile.getint('gesture_recognition','hand_ht')

        # 計算入力
        self.r_wrist = np.zeros((axis))
        self.l_wrist = np.zeros((axis))
        self.r_elbow = np.zeros((axis))
        self.l_elbow = np.zeros((axis))
        self.r_handtip = np.zeros((axis))
        self.l_handtip = np.zeros((axis))
        self.r_hand = np.zeros((axis))
        self.l_hand = np.zeros((axis))
        self.r_thumb = np.zeros((axis))
        self.l_thumb = np.zeros((axis))
        self.head = np.zeros((axis))
        self.chest = np.zeros((axis))
        self.naval = np.zeros((axis))
        self.nose = np.zeros((axis))

        self.is_r_shuriken = 0
        self.is_l_shuriken = 0
        self.throw_shuriken = 0

        # These control whether a shuriken is available to throw
        self.reload_status_l = 1
        self.reload_status_r = 1

        # Get config file stuff
        # self.seed_naval_z_dist = inifile.getint('gesture_recognition','seed_naval_z_dist')
        self.toss_start_limit = 0.9
        #Generate the screen as a plan to determine the intercept point with the push direction
        self.screen_point_1 = np.array([0, 0, -1000]) #Temporary dummy values - change to real values when known
        self.screen_point_2 = np.array([50, 0, -1000])
        self.screen_point_3 = np.array([0, 50, -1000])
        self.screen_plane = py.plane.create_from_points(self.screen_point_1, self.screen_point_2, self.screen_point_3)


    def calculate(self,
                  r_wrist=np.zeros(3),
                  l_wrist=np.zeros(3),
                  r_elbow=np.zeros(3),
                  l_elbow=np.zeros(3),
                  r_handtip=np.zeros(3),
                  l_handtip=np.zeros(3),
                  r_hand=np.zeros(3),
                  l_hand=np.zeros(3),
                  r_shoulder=np.zeros(3),
                  l_shoulder=np.zeros(3),
                  head=np.zeros(3),
                  chest=np.zeros(3),
                  naval=np.zeros(3),
                  nose=np.zeros(3),
                  is_data=False):
        # 初期値
        self.is_r_shuriken = 0
        self.is_l_shuriken = 0
        self.throw_shuriken = 0

        x_idx = 0
        y_idx = 1
        z_idx = 2

        if (is_data):
            #Prelim Calcs
            #Arm length
            r_wr_len = math.sqrt((r_wrist[0]-r_elbow[0])**2 + (r_wrist[1]-r_elbow[1])**2 + (r_wrist[2]-r_elbow[2])**2)
            l_wr_len = math.sqrt((l_wrist[0]-l_elbow[0])**2 + (l_wrist[1]-l_elbow[1])**2 + (l_wrist[2]-l_elbow[2])**2)
            #Elbow to chest
            r_el_sh_len = math.sqrt((r_elbow[0]-r_shoulder[0])**2 + (r_elbow[1]-r_shoulder[1])**2 + (r_elbow[2]-r_shoulder[2])**2)
            l_el_sh_len = math.sqrt((l_elbow[0]-l_shoulder[0])**2 + (l_elbow[1]-l_shoulder[1])**2 + (l_elbow[2]-l_shoulder[2])**2)
            # Trigger arm length
            r_max_arm_length = (r_wr_len + r_el_sh_len) * self.toss_start_limit
            l_max_arm_length = (l_wr_len + l_el_sh_len) * self.toss_start_limit
            # Current arm length
            r_curr_arm_length = math.sqrt((r_wrist[0]-r_shoulder[0])**2 + (r_wrist[1]-r_shoulder[1])**2 + (r_wrist[2]-r_shoulder[2])**2)
            l_curr_arm_length = math.sqrt((l_wrist[0]-l_shoulder[0])**2 + (l_wrist[1]-l_shoulder[1])**2 + (l_wrist[2]-l_shoulder[2])**2)
            # Arm ratio
            r_ratio = r_curr_arm_length/r_max_arm_length
            l_ratio = l_curr_arm_length/l_max_arm_length

            # Wrist to head length - Used to determine start of throw
            wr_hd_length_r = math.sqrt((r_wrist[x_idx] - head[x_idx]) ** 2 + (r_wrist[y_idx] - head[y_idx]) ** 2 + (r_wrist[z_idx] - head[z_idx]) ** 2)
            wr_hd_length_l = math.sqrt((l_wrist[x_idx] - head[x_idx]) ** 2 + (l_wrist[y_idx] - head[y_idx]) ** 2 + (l_wrist[z_idx] - head[z_idx]) ** 2)
            # Elbow to head length - Used to determine start of throw
            el_hd_length_r = math.sqrt((r_elbow[x_idx] - head[x_idx]) ** 2 + (r_elbow[y_idx] - head[y_idx]) ** 2 + (r_elbow[z_idx] - head[z_idx]) ** 2)
            el_hd_length_l = math.sqrt((l_elbow[x_idx] - head[x_idx]) ** 2 + (l_elbow[y_idx] - head[y_idx]) ** 2 + (l_elbow[z_idx] - head[z_idx]) ** 2)

            #Throw Direction
            throw_dir_lx = l_wrist[0] - head[0]
            throw_dir_ly = l_wrist[1] - head[1]
            throw_dir_lz = l_wrist[2] - head[2]
            l_throw_dir = np.array([throw_dir_lx,throw_dir_ly,throw_dir_lz])

            throw_dir_rx = r_wrist[0] - head[0]
            throw_dir_ry = r_wrist[1] - head[1]
            throw_dir_rz = r_wrist[2] - head[2]
            r_throw_dir = np.array([throw_dir_rx,throw_dir_ry,throw_dir_rz])

            # Determine intersection between push and screen
            throw_dir_ray_l = py.ray.create(head, l_throw_dir)
            intersect_l = py.geometric_tests.ray_intersect_plane(throw_dir_ray_l, self.screen_plane, False)
            throw_dir_ray_r = py.ray.create(head, r_throw_dir)
            intersect_r = py.geometric_tests.ray_intersect_plane(throw_dir_ray_r, self.screen_plane, False)


            # Reload Status Calculations
            if self.reload_status_r == 0:
                if wr_hd_length_r <= el_hd_length_r:
                    self.reload_status_r = 1

            if self.reload_status_l == 0:
                if wr_hd_length_l <= el_hd_length_l:
                    self.reload_status_l = 1

            # Determine if a shuriken is thrown
            if self.is_r_shuriken == 0 and self.reload_status_r == 1:
                if r_wrist[y_idx] >= naval[y_idx]:
                    if wr_hd_length_r >= el_hd_length_r:
                        if r_ratio >1:
                            self.is_r_shuriken = 1
                            self.reload_status_r = 0
            if self.is_l_shuriken == 0 and self.reload_status_l == 1:
                if l_wrist[y_idx] >= naval[y_idx]:
                    if wr_hd_length_l >= el_hd_length_l:
                        if l_ratio >1:
                            self.is_l_shuriken = 1
                            self.reload_status_l = 0
            up_val = []
            up_val.append(self.is_l_shuriken)
            up_val.append(self.is_r_shuriken)
            self.throw_shuriken = max(up_val)

        return self.throw_shuriken, self.is_r_shuriken, self.is_l_shuriken #For direction we can pass screen intersect and direction array
