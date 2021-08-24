# coding:utf-8
import numpy as np

class DetectFaceToHandDirection:
    def __init__(self):
        self.axis = axis = 3
        self.axis_tank = ['x', 'y', 'z']

        self.zero = np.zeros((axis))
        self.bace_h = np.array([0.0, 0.0, 1.0])
        self.bace_v = np.array([0.0, 1.0, 0.0])

        self.direction_horizontal_r = 0.0
        self.direction_horizontal_l = 0.0
        self.direction_vertical_r = 0.0
        self.direction_vertical_l = 0.0
        self.direction_bace_x = 0.0
        self.direction_bace_y = 0.0
        self.direction_bace_z = 0.0
        self.direction_bace_right_eyes_x = 0.0
        self.direction_bace_right_eyes_y = 0.0
        self.direction_bace_right_eyes_z = 0.0
        self.direction_bace_left_eyes_x = 0.0
        self.direction_bace_left_eyes_y = 0.0
        self.direction_bace_left_eyes_z = 0.0

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

    def set_horizontal_input(self, body_dict):
        l_hand = np.zeros((self.axis))
        r_hand = np.zeros((self.axis))
        l_eyes = np.zeros((self.axis))
        r_eyes = np.zeros((self.axis))
        head = np.zeros((self.axis))
        nose = np.zeros((self.axis))
        axt = self.axis_tank
        for ax in range(self.axis):
            l_hand[ax] = body_dict['l_hand'][axt[ax]]
            r_hand[ax] = body_dict['r_hand'][axt[ax]]
            l_eyes[ax] = body_dict['l_eyes'][axt[ax]]
            r_eyes[ax] = body_dict['r_eyes'][axt[ax]]
            head[ax] = body_dict['head'][axt[ax]]
            nose[ax] = body_dict['nose'][axt[ax]]

        self.toward_horizontal_vector_r = (r_hand - r_eyes)
        self.toward_horizontal_vector_l = (l_hand - l_eyes)
        self.toward_horizontal_vector_head = (nose - head)
        self.bace_horizontal_vector = self.bace_h

    def set_horizontal_input_for_test(self, 
                                      toward_vector_r, 
                                      toward_vector_l, 
                                      toward_vector_head, 
                                      bace_vector):
        self.toward_horizontal_vector_r = toward_vector_r
        self.toward_horizontal_vector_l = toward_vector_l
        self.toward_horizontal_vector_head = toward_vector_head
        self.bace_horizontal_vector = bace_vector

    def calculate_horizontal_direction(self):
        toward_vector_r = self.toward_horizontal_vector_r
        toward_vector_l = self.toward_horizontal_vector_l
        toward_vector_head = self.toward_horizontal_vector_head
        bace_vector = self.bace_horizontal_vector

        # right hand
        h_hand_angle_r = self.calculate_2d_angle(
            toward_vector_r, bace_vector, axis1=0, axis2=2)
        hand_ward_r = self.calculate_horizontal_angle(
            h_hand_angle_r, toward_vector_r)
        self.direction_horizontal_r = hand_ward_r

        # left hand
        h_hand_angle_l = self.calculate_2d_angle(
            toward_vector_l, bace_vector, axis1=0, axis2=2)
        hand_ward_l = self.calculate_horizontal_angle(
            h_hand_angle_l, toward_vector_l)
        self.direction_horizontal_l = hand_ward_l

        # head
        h_head_angle = self.calculate_2d_angle(
            toward_vector_head, bace_vector, axis1=0, axis2=2)
        head_ward = self.calculate_horizontal_angle(
            h_head_angle, toward_vector_head)
        self.direction_horizontal = head_ward

    def calculate_horizontal_angle(self, h_hand_angle, toward_vector):
        if(toward_vector[0]>=0):
            if(toward_vector[2]>=0):
                hand_ward = h_hand_angle
            else:
                hand_ward = h_hand_angle - 360
        else:
            if(toward_vector[2]>=0):
                hand_ward = (-1) * h_hand_angle
            else:
                hand_ward = (-1) * (h_hand_angle)
        return hand_ward

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

    def set_vartical_input(self, body_dict):
        l_hand = np.zeros((self.axis))
        r_hand = np.zeros((self.axis))
        l_eyes = np.zeros((self.axis))
        r_eyes = np.zeros((self.axis))
        head = np.zeros((self.axis))
        nose = np.zeros((self.axis))
        axt = self.axis_tank
        for ax in range(self.axis):
            l_hand[ax] = body_dict['l_hand'][axt[ax]]
            r_hand[ax] = body_dict['r_hand'][axt[ax]]
            l_eyes[ax] = body_dict['l_eyes'][axt[ax]]
            r_eyes[ax] = body_dict['r_eyes'][axt[ax]]
            head[ax] = body_dict['head'][axt[ax]]
            nose[ax] = body_dict['nose'][axt[ax]]

        self.toward_vartical_vector_r = (r_hand - r_eyes)
        self.toward_vartical_vector_l = (l_hand - l_eyes)
        self.toward_vartical_vector_head = (nose - head)
        self.bace_vartical_vector = self.bace_v

    def set_vartical_input_for_test(self, 
                                    toward_vector_r, 
                                    toward_vector_l, 
                                    toward_vector_head, 
                                    bace_vector):
        self.toward_vartical_vector_r = toward_vector_r
        self.toward_vartical_vector_l = toward_vector_l
        self.toward_vartical_vector_head = toward_vector_head
        self.bace_vartical_vector = bace_vector

    def calculate_vartical_direction(self):
        toward_vector_r = self.toward_vartical_vector_r
        toward_vector_l = self.toward_vartical_vector_l
        toward_vector_head = self.toward_vartical_vector_head
        bace_vector = self.bace_vartical_vector

        # right hand
        v_hand_angle_r = self.calculate_3d_angle(
            toward_vector_r, bace_vector)
        hand_ward_r = self.calculate_vartical_angle(v_hand_angle_r)
        self.direction_vertical_r = hand_ward_r

        # left hand
        v_hand_angle_l = self.calculate_3d_angle(
            toward_vector_l, bace_vector)
        hand_ward_l = self.calculate_vartical_angle(v_hand_angle_l)
        self.direction_vertical_l = hand_ward_l

        # head
        v_hand_angle_head = self.calculate_3d_angle(
            toward_vector_head, bace_vector)
        hand_ward_head = self.calculate_vartical_angle(v_hand_angle_head)
        self.direction_vertical_head = hand_ward_head

    def calculate_vartical_angle(self, v_hand_angle):
        hand_ward = 90 - v_hand_angle
        if(hand_ward > 180):
            hand_ward = hand_ward - 360
        elif(hand_ward < -180):
            hand_ward = hand_ward + 360
        return hand_ward

    def calculate_bace(self, body_dict):
        l_eyes = np.zeros((self.axis))
        r_eyes = np.zeros((self.axis))
        axt = self.axis_tank
        for ax in range(self.axis):
            l_eyes[ax] = body_dict['l_eyes'][axt[ax]]
            r_eyes[ax] = body_dict['r_eyes'][axt[ax]]
        bace = (r_eyes + l_eyes)/2.
        self.direction_bace_x = bace[0]
        self.direction_bace_y = bace[1]
        self.direction_bace_z = bace[2]
        self.direction_bace_right_eyes_x = r_eyes[0]
        self.direction_bace_right_eyes_y = r_eyes[1]
        self.direction_bace_right_eyes_z = r_eyes[2]
        self.direction_bace_left_eyes_x = l_eyes[0]
        self.direction_bace_left_eyes_y = l_eyes[1]
        self.direction_bace_left_eyes_z = l_eyes[2]

    def set_blank(self):
        self.direction_horizontal_r = 0.0
        self.direction_horizontal_l = 0.0
        self.direction_vertical_r = 0.0
        self.direction_vertical_l = 0.0
        self.direction_bace_x = 0.0
        self.direction_bace_y = 0.0
        self.direction_bace_z = 0.0
        self.direction_bace_right_eyes_x = 0.0
        self.direction_bace_right_eyes_y = 0.0
        self.direction_bace_right_eyes_z = 0.0
        self.direction_bace_left_eyes_x = 0.0
        self.direction_bace_left_eyes_y = 0.0
        self.direction_bace_left_eyes_z = 0.0

    def set_dict(self):
        output_dict = {}
        output_dict['right_hand'] = {}
        output_dict['right_hand']['direction_horizontal'] = self.direction_horizontal_r
        output_dict['right_hand']['direction_vertical'] = self.direction_vertical_r
        output_dict['right_hand']['direction_bace_x'] = self.direction_bace_right_eyes_x
        output_dict['right_hand']['direction_bace_y'] = self.direction_bace_right_eyes_y
        output_dict['right_hand']['direction_bace_z'] = self.direction_bace_right_eyes_z
        output_dict['left_hand'] = {}
        output_dict['left_hand']['direction_horizontal'] = self.direction_horizontal_l
        output_dict['left_hand']['direction_vertical'] = self.direction_vertical_l
        output_dict['left_hand']['direction_bace_x'] = self.direction_bace_left_eyes_x
        output_dict['left_hand']['direction_bace_y'] = self.direction_bace_left_eyes_y
        output_dict['left_hand']['direction_bace_z'] = self.direction_bace_left_eyes_z
        output_dict['head'] = {}
        output_dict['head']['direction_horizontal'] = self.direction_horizontal_l
        output_dict['head']['direction_vertical'] = self.direction_vertical_l
        output_dict['head']['direction_bace_x'] = self.direction_bace_x
        output_dict['head']['direction_bace_y'] = self.direction_bace_y
        output_dict['head']['direction_bace_z'] = self.direction_bace_z
        self.output_data = output_dict

    def Calculate(self,
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
                           h_toward_vector_r=np.zeros(3),
                           h_toward_vector_l=np.zeros(3),
                           h_toward_vector_head=np.zeros(3),
                           h_bace_vector=np.zeros(3),
                           v_toward_vector_r=np.zeros(3),
                           v_toward_vector_l=np.zeros(3),
                           v_toward_vector_head=np.zeros(3),
                           v_bace_vector=np.zeros(3),
                           is_data=False):
        if (is_data):
            self.set_horizontal_input_for_test(
                                            h_toward_vector_r,
                                            h_toward_vector_l, 
                                            h_toward_vector_head, 
                                            h_bace_vector)
            self.calculate_horizontal_direction()

            self.set_vartical_input_for_test(
                                            v_toward_vector_r, 
                                            v_toward_vector_l, 
                                            v_toward_vector_head, 
                                            v_bace_vector)
            self.calculate_vartical_direction()

            self.calculate_bace(body_dict)
        else:
            self.set_blank()
        self.set_dict()
