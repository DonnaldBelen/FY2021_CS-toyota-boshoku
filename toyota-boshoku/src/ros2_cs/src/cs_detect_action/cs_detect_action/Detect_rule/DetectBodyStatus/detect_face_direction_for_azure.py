# coding:utf-8
import numpy as np
import configparser
import json
import ast
import os

class DetectFaceDirection:
    def __init__(self):
        self.axis = axis = 3
        self.axis_tank = ['x', 'y', 'z']

        body_conf = configparser.ConfigParser()
        body_conf.read(
            os.path.dirname(
                os.path.abspath(__file__)) +
            '/config/BodyStatusConfig.ini',
            'UTF-8')
        cols_id = int(body_conf.get('look_config', 'cols_select_id'))
        self.cols_tank = cols_tank = ast.literal_eval(body_conf.get('look_config', 'cols_tank'))
        #['horizontal_bace_joint', 'horizontal_toward_joint', 'vertical_bace_joint', 'vertical_toward_joint']
        if(len(cols_tank)-1 < cols_id):
            cols_id = 0
        self.cols = cols = cols_tank[cols_id]
        self.alg_code = cols[0]
        self.alg_name = cols[1]+'_'+cols[2]+'_'+cols[3]+'_'+cols[4]
        self.bace_joint_h = cols[1]
        self.toward_joint_h = cols[2]
        self.bace_joint_v = cols[3]
        self.toward_joint_v = cols[4]

        self.zero = np.zeros((axis))
        self.bace_h = np.array([0.0, 0.0, 1.0])
        self.bace_v = np.array([0.0, 1.0, 0.0])

        self.face_direction_horizontal = 0.0
        self.face_direction_vertical = 0.0
        self.face_direction_bace_x = 0.0
        self.face_direction_bace_y = 0.0
        self.face_direction_bace_z = 0.0
        self.face_direction_bace_right_eyes_x = 0.0
        self.face_direction_bace_right_eyes_y = 0.0
        self.face_direction_bace_right_eyes_z = 0.0
        self.face_direction_bace_left_eyes_x = 0.0
        self.face_direction_bace_left_eyes_y = 0.0
        self.face_direction_bace_left_eyes_z = 0.0
        self.is_front_ward = False

    def calculate_2d_angle(self, toward, bace, axis1=0, axis2=2):
        A = toward
        B = bace
        A = np.array([A[axis1], A[axis2]])
        B = np.array([B[axis1], B[axis2]])
        dot_xz = np.inner(A, B)
        s = np.linalg.norm(A)
        t = np.linalg.norm(B)
        if(s*t != 0):
            cos = dot_xz / (s * t)
        else:
            cos = dot_xz / 0.0001
        rad = np.arccos(cos)
        theta = rad * 180 / np.pi
        return theta

    def calculate_3d_angle(self, toward, bace,):
        A = toward
        B = bace
        dot = np.inner(A, B)
        s = np.linalg.norm(A)
        t = np.linalg.norm(B)
        cos = dot / (s * t)
        rad = np.arccos(cos)
        theta = rad * 180 / np.pi
        return theta

    def calculate_horizontal_direction(self):
        toward_vector = self.toward_horizontal_vector
        bace_vector = self.bace_horizontal_vector
        is_front_ward = self.is_front_ward

        h_face_angle = self.calculate_2d_angle(
            toward_vector, bace_vector, axis1=0, axis2=2)
        if not (is_front_ward):
            if(toward_vector[0]>=0):
                if(toward_vector[2]>=0):
                    face_ward = h_face_angle + 90
                else:
                    face_ward = h_face_angle + 90 - 360
            else:
                if(toward_vector[2]>=0):
                    face_ward = (-1) * h_face_angle + 90
                else:
                    face_ward = (-1) * (h_face_angle) + 90
        else:
            if(toward_vector[0]>=0):
                if(toward_vector[2]>=0):
                    face_ward = h_face_angle
                else:
                    face_ward = h_face_angle - 360
            else:
                if(toward_vector[2]>=0):
                    face_ward = (-1) * h_face_angle
                else:
                    face_ward = (-1) * (h_face_angle)
        self.face_direction_horizontal = face_ward

    def calculate_vartical_direction(self):
        toward_vector = self.toward_vartical_vector
        bace_vector = self.bace_vartical_vector

        v_face_angle = self.calculate_3d_angle(
            toward_vector, bace_vector)

        face_ward = 90 - v_face_angle
        if(face_ward > 180):
            face_ward = face_ward - 360
        elif(face_ward < -180):
            face_ward = face_ward + 360
        self.face_direction_vertical = face_ward

    def calculate_bace(self, body_dict):
        l_eyes = np.zeros((self.axis))
        r_eyes = np.zeros((self.axis))
        axt = self.axis_tank
        for ax in range(self.axis):
            l_eyes[ax] = body_dict['l_eyes'][axt[ax]]
            r_eyes[ax] = body_dict['r_eyes'][axt[ax]]
        bace = (r_eyes + l_eyes)/2.
        self.face_direction_bace_x = bace[0]
        self.face_direction_bace_y = bace[1]
        self.face_direction_bace_z = bace[2]
        self.face_direction_bace_right_eyes_x = r_eyes[0]
        self.face_direction_bace_right_eyes_y = r_eyes[1]
        self.face_direction_bace_right_eyes_z = r_eyes[2]
        self.face_direction_bace_left_eyes_x = l_eyes[0]
        self.face_direction_bace_left_eyes_y = l_eyes[1]
        self.face_direction_bace_left_eyes_z = l_eyes[2]

    def set_blank(self):
        self.face_direction_horizontal = 0.0
        self.face_direction_vertical = 0.0
        self.face_direction_bace_x = 0.0
        self.face_direction_bace_y = 0.0
        self.face_direction_bace_z = 0.0
        self.face_direction_bace_right_eyes_x = 0.0
        self.face_direction_bace_right_eyes_y = 0.0
        self.face_direction_bace_right_eyes_z = 0.0
        self.face_direction_bace_left_eyes_x = 0.0
        self.face_direction_bace_left_eyes_y = 0.0
        self.face_direction_bace_left_eyes_z = 0.0

    def set_dict(self):
        output_dict = {}
        output_dict['face_direction_horizontal'] = self.face_direction_horizontal
        output_dict['face_direction_vertical'] = self.face_direction_vertical
        output_dict['face_direction_bace_x'] = self.face_direction_bace_x
        output_dict['face_direction_bace_y'] = self.face_direction_bace_y
        output_dict['face_direction_bace_z'] = self.face_direction_bace_z
        output_dict['face_direction_bace_right_eyes_x'] = self.face_direction_bace_right_eyes_x
        output_dict['face_direction_bace_right_eyes_y'] = self.face_direction_bace_right_eyes_y
        output_dict['face_direction_bace_right_eyes_z'] = self.face_direction_bace_right_eyes_z
        output_dict['face_direction_bace_left_eyes_x'] = self.face_direction_bace_left_eyes_x
        output_dict['face_direction_bace_left_eyes_y'] = self.face_direction_bace_left_eyes_y
        output_dict['face_direction_bace_left_eyes_z'] = self.face_direction_bace_left_eyes_z
        output_dict['face_direction_alg_code'] = self.alg_code
        output_dict['face_direction_alg_select'] = self.alg_name

        self.output_data = output_dict

    def get_vector(self, col, body_dict):
        vector = np.zeros((self.axis))
        axt = self.axis_tank
        for ax in range(self.axis):
            if(col == 'm_eyes'):
                vector[ax] = (body_dict['l_eyes'][axt[ax]] +
                              body_dict['r_eyes'][axt[ax]])/2.
            elif(col == 'm_ear'):
                vector[ax] = (body_dict['l_ear'][axt[ax]] +
                              body_dict['r_ear'][axt[ax]])/2.
            else:
                vector[ax] = body_dict[col][axt[ax]]
        return vector

    def set_vector_input(self, body_dict):
        self.is_front_ward = True
        bace_h_vector = self.get_vector(self.bace_joint_h, body_dict)
        bace_v_vector = self.get_vector(self.bace_joint_v, body_dict)
        toward_h_vector = self.get_vector(self.toward_joint_h, body_dict)
        toward_v_vector = self.get_vector(self.toward_joint_v, body_dict)
        
        self.set_vartical_input_for_test(
            (toward_v_vector - bace_v_vector), self.bace_v)
        self.set_horizontal_input_for_test(
            (toward_h_vector - bace_h_vector), self.bace_h, is_front_ward=self.is_front_ward)

    def set_horizontal_input(self, body_dict):
        l_eyes = np.zeros((self.axis))
        r_eyes = np.zeros((self.axis))
        nose = np.zeros((self.axis))
        head = np.zeros((self.axis))
        r_ear = np.zeros((self.axis))
        l_ear = np.zeros((self.axis))
        axt = self.axis_tank
        for ax in range(self.axis):
            l_eyes[ax] = body_dict['l_eyes'][axt[ax]]
            r_eyes[ax] = body_dict['r_eyes'][axt[ax]]
            nose[ax] = body_dict['nose'][axt[ax]]
            head[ax] = body_dict['head'][axt[ax]]
            l_ear[ax] = body_dict['l_ear'][axt[ax]]
            r_ear[ax] = body_dict['r_ear'][axt[ax]]
        self.is_front_ward = True
        self.toward_horizontal_vector = (l_eyes + r_eyes)/2. - (r_ear + l_ear)/2.
        self.bace_horizontal_vector = self.bace_h

    def set_vartical_input(self, body_dict):
        l_eyes = np.zeros((self.axis))
        r_eyes = np.zeros((self.axis))
        nose = np.zeros((self.axis))
        r_ear = np.zeros((self.axis))
        l_ear = np.zeros((self.axis))
        axt = self.axis_tank
        for ax in range(self.axis):
            l_eyes[ax] = body_dict['l_eyes'][axt[ax]]
            r_eyes[ax] = body_dict['r_eyes'][axt[ax]]
            nose[ax] = body_dict['nose'][axt[ax]]
            l_ear[ax] = body_dict['l_ear'][axt[ax]]
            r_ear[ax] = body_dict['r_ear'][axt[ax]]

        self.toward_vartical_vector = nose - (l_ear + r_ear)/2.
        self.bace_vartical_vector = self.bace_v

    def set_horizontal_input_for_test(self, toward_vector, bace_vector, is_front_ward=False):
        self.is_front_ward = is_front_ward
        self.toward_horizontal_vector = toward_vector
        self.bace_horizontal_vector = bace_vector

    def set_vartical_input_for_test(self, toward_vector, bace_vector):
        self.toward_vartical_vector = toward_vector
        self.bace_vartical_vector = bace_vector

    def set_cols(self, cols_id):
        #['horizontal_bace_joint', 'horizontal_toward_joint', 'vertical_bace_joint', 'vertical_toward_joint']
        if(len(self.cols_tank)-1 < cols_id):
            cols_id = 0
        self.cols = cols = self.cols_tank[cols_id]
        self.alg_code = cols[0]
        self.alg_name = cols[1]+'_'+cols[2]+'_'+cols[3]+'_'+cols[4]
        self.bace_joint_h = cols[1]
        self.toward_joint_h = cols[2]
        self.bace_joint_v = cols[3]
        self.toward_joint_v = cols[4]

    def Calculate(self,
                  body_dict,
                  is_data=False):
        if (is_data):
            self.set_vector_input(body_dict)
            self.calculate_horizontal_direction()
            self.calculate_vartical_direction()
            self.calculate_bace(body_dict)
        else:
            self.set_blank()
        self.set_dict()

    def Calculate_old(self,
                      body_dict,
                      is_data=False):
        if (is_data):
            self.set_horizontal_input(body_dict)
            self.calculate_horizontal_direction()

            self.set_vartical_input(body_dict)
            self.calculate_vartical_direction()

            self.calculate_bace(body_dict)
        else:
            self.set_blank()
        self.set_dict()

    def Calculate_for_test(self,
                           body_dict,
                           h_toward_vector=np.zeros(3),
                           h_bace_vector=np.zeros(3),
                           v_toward_vector=np.zeros(3),
                           v_bace_vector=np.zeros(3),
                           is_data=False,
                           is_front_ward=False):
        if (is_data):
            self.set_horizontal_input_for_test(
                h_toward_vector, h_bace_vector, is_front_ward=is_front_ward)
            self.calculate_horizontal_direction()

            self.set_vartical_input_for_test(v_toward_vector, v_bace_vector)
            self.calculate_vartical_direction()

            self.calculate_bace(body_dict)
        else:
            self.set_blank()
        self.set_dict()
