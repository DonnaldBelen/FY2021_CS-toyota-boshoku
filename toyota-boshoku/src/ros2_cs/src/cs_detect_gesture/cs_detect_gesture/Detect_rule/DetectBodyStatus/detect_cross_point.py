# coding:utf-8
from decimal import Decimal, ROUND_HALF_UP, ROUND_HALF_EVEN
import numpy as np

class DetectCrossPoint:
    def __init__(self, object_tank=[[400, 400, 500],
                                    [400,   0, 500],
                                    [-400,   0, 500],
                                    [-400, 400, 500],
                                    ['screen'], ]):
        self.axis = axis = 3
        self.zero = np.zeros((axis))
        self.object_tank = object_tank
        self.look_point_x = 0.0
        self.look_point_y = 0.0
        self.look_point_z = 0.0
        self.look_object = 'none'
        self.point_box = []

    def set_input_data(self, 
                       h_dir=0.0,
                       v_dir=0.0,
                       bace_x=0.0,
                       bace_y=0.0,
                       bace_z=0.0,):
        self.face_direction_horizontal = np.deg2rad(h_dir)
        self.face_direction_vertical = np.deg2rad(90-v_dir)
        self.face_direction_bace_x = bace_z
        self.face_direction_bace_y = bace_x
        self.face_direction_bace_z = bace_y
        self.look_point_x = 0.0
        self.look_point_y = 0.0
        self.look_point_z = 0.0
        self.look_object = 'none'
        self.point_box = []

    def calculate_object_line(self, object_points=[[ 400, 400, 500], 
                                                   [ 400,   0, 500], 
                                                   [-400,   0, 500], 
                                                   [-400, 400, 500],
                                                   ['screen'],]):
        x0 = object_points[0][2]
        y0 = object_points[0][0]
        z0 = object_points[0][1]
        x1 = object_points[1][2]
        y1 = object_points[1][0]
        z1 = object_points[1][1]
        x2 = object_points[2][2]
        y2 = object_points[2][0]
        z2 = object_points[2][1]

        # 三点から2つのベクトルを出し、その外積で法線ベクトルを求めることで、a,b,cを算出
        self.a = (y1 - y0) * (z2 - z0) - (y2 - y0) * (z1 - z0)
        self.b = (z1 - z0) * (x2 - x0) - (z2 - z0) * (x1 - x0)
        self.c = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)

        # dは1点代入で算出
        self.d = (self.a * x0 + self.b * y0 + self.c * z0 ) * (-1)

    def calculate_looking_line(self):
        self.x0 = self.face_direction_bace_x
        self.y0 = self.face_direction_bace_y
        self.z0 = self.face_direction_bace_z
        self.l = np.sin(self.face_direction_vertical) * \
            np.cos(self.face_direction_horizontal)
        self.m = np.sin(self.face_direction_vertical) * \
            np.sin(self.face_direction_horizontal)
        self.n = np.cos(self.face_direction_vertical)

    def calculate_cross_point(self, object_points=[[ 400, 400, 500],
                                                   [ 400,   0, 500],
                                                   [-400,   0, 500],
                                                   [-400, 400, 500],
                                                   ['screen'],]):
        upper = (self.a * self.x0 + self.b * self.y0 + self.c * self.z0 + self.d)
        low = (self.a * self.l + self.b * self.m + self.c * self.n )
        if(low != 0):
            t = (-1) * upper / low
        else:
            t = 0

        # is front?
        if(t <= 0):
            is_front = False
        else:
            is_front = True

        look_point_x = self.x0 + t * self.l
        look_point_y = self.y0 + t * self.m
        look_point_z = self.z0 + t * self.n

        look_point_x = float(Decimal(str(look_point_x)).quantize(
            Decimal('0.01'), rounding=ROUND_HALF_UP))
        look_point_y = float(Decimal(str(look_point_y)).quantize(
            Decimal('0.01'), rounding=ROUND_HALF_UP))
        look_point_z = float(Decimal(str(look_point_z)).quantize(
            Decimal('0.01'), rounding=ROUND_HALF_UP))

        x_max = np.max([object_points[0][2], object_points[1][2],
                        object_points[2][2], object_points[3][2],])
        x_min = np.min([object_points[0][2], object_points[1][2],
                        object_points[2][2], object_points[3][2], ])
        on_x = x_min <= look_point_x and look_point_x <= x_max

        y_max = np.max([object_points[0][0], object_points[1][0],
                        object_points[2][0], object_points[3][0], ])
        y_min = np.min([object_points[0][0], object_points[1][0],
                        object_points[2][0], object_points[3][0], ])
        on_y = y_min <= look_point_y and look_point_y <= y_max

        z_max = np.max([object_points[0][1], object_points[1][1],
                        object_points[2][1], object_points[3][1], ])
        z_min = np.min([object_points[0][1], object_points[1][1],
                        object_points[2][1], object_points[3][1], ])
        on_z = z_min <= look_point_z and look_point_z <= z_max

        is_on_object = all([on_x, on_y, on_z, is_front])
        if(is_on_object):
            self.look_point_x = look_point_y
            self.look_point_y = look_point_z
            self.look_point_z = look_point_x
            self.look_object = look_object = object_points[4][0]
            self.point_box.append(
                [self.look_point_x, self.look_point_y, self.look_point_z, self.look_object,])

    def select_closest_object(self):
        if(len(self.point_box)!=0):
            a = 99999
            bace_x = self.face_direction_bace_x
            bace_y = self.face_direction_bace_y
            bace_z = self.face_direction_bace_z
            for point_box in self.point_box:
                ax = point_box[0]
                ay = point_box[1]
                az = point_box[2]
                obj = point_box[3]
                distance = np.sqrt((ax - bace_x) * (ax - bace_x) +
                                   (ay - bace_y) * (ay - bace_y) + 
                                   (az - bace_z) * (az - bace_z))
                if(a >= distance):
                    a = distance
                    self.look_point_x = ax
                    self.look_point_y = ay
                    self.look_point_z = az
                    self.look_object = obj
                    self.look_distance = distance

    def Calculate(self,
                  h_dir=0.0,
                  v_dir=0.0,
                  bace_x=0.0,
                  bace_y=0.0,
                  bace_z=0.0,):

        self.set_input_data(h_dir=h_dir,
                            v_dir=v_dir,
                            bace_x=bace_x,
                            bace_y=bace_y,
                            bace_z=bace_z,)

        self.calculate_looking_line()

        for object_pos in self.object_tank:

            self.calculate_object_line(object_pos)

            self.calculate_cross_point(object_pos)

        self.select_closest_object()
