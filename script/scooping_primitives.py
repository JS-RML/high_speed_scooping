#!/usr/bin/python3
import time
from math import sin, cos, radians, pi
import numpy as np
import yaml
import urx
import math3d as m3d
import odrive



class HighSpeedScooping:

    def __init__(self, robot, gripper, config_name):
        self.ur = robot
        self.ddh = gripper
        config_file = config_name + ".yaml"
        with open("../config/"+config_file, 'r') as stream:
            try:
                print('reading scooping config...')
                config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        
        self.obj_l = config['object_length']
        self.obj_t = config['object_thickness']
        self.theta = config['gripper_tilt']
        self.grip_h = config['gripper_height']
        self.cont_dist = config['contact_distance']
        self.fg_dist = config['finger_position'] * self.obj_l
        self.tb_dist = config['thumb_prescoop_position'] * self.obj_l
        self.center_dist = config['gripper_center'] * self.obj_l
        self.T_t_g = np.array(config['T_tool_gripper'])
        self.P_g_L = np.array(config['P_gripper_MotorL'])
        self.P_g_R = np.array(config['P_gripper_MotorR'])
        self.tcp_vel = config['tcp_vel']
        self.tcp_acc = config['tcp_acc']


    def initialize_gripper_pose(self, object_2D_pose):
        '''Initial robot pose before smack and scoop with the gripper pose setting above the object
        Parameters:
            object_2D_pose(tuple): (x_obj, y_obj, q_obj) object pose in x-y plane of ur10 base frame 
                x_obj(m): x coordinate of object's center
                y_obj(m): y coordinate of object's center
                q_obj(degree): angle of object's longer center line relative to x axis of ur10 base frame
        Returns:
        
        '''
        x_obj, y_obj, q_obj = object_2D_pose
        
        # ======set fingers position according to scooping model======
        # compute F, T position in gripper frame
        y_g_F = -(self.fg_dist - self.center_dist) * sin(radians(self.theta))
        x_g_F = self.cont_dist + (self.fg_dist - self.center_dist) * cos(radians(self.theta))
        P_g_F = np.array([x_g_F, y_g_F, 0])
        y_g_T = (self.center_dist + self.tb_dist) * sin(radians(self.theta))
        x_g_T = self.cont_dist - (self.center_dist + self.tb_dist) * cos(radians(self.theta))
        P_g_T = np.array([x_g_T, y_g_T, 0])
        # F, T target position in their motor frame
        P_L_F = P_g_F - self.P_g_L
        P_R_T = P_g_T - self.P_g_R
        self.ddh.set_left_tip(tuple(P_L_F[:-1]))
        self.ddh.set_right_tip(tuple(P_R_T[:-1]))

        # ======set gripper pose according to object pose and tilt angle======
        # F position in tool frame
        P_t_F = np.matmul(self.T_t_g, np.append(P_g_F, 1))[:-1]
        # set F as tcp
        P_t_F = P_t_F / 1000
        self.ur.set_tcp(np.append(P_t_F, [0,0,0]))
        # set gripper initial pose
        init_pose = m3d.Transform()
        # work in -ve x and +ve y region only
        init_pose.pos.x = x_obj - (self.fg_dist - self.center_dist) * cos(radians(q_obj)) / 1000
        init_pose.pos.y = y_obj + (self.fg_dist - self.center_dist) * sin(radians(q_obj)) / 1000
        init_pose.pos.z = self.grip_h
        init_pose.orient.rotate_xb(pi)
        init_pose.orient.rotate_zt(radians(90-q_obj))
        init_pose.orient.rotate_xt(radians(90-self.theta))
        print("Setting pose: ")
        print(init_pose.pose_vector)
        self.ur.set_pose(init_pose, self.tcp_vel, self.tcp_acc)

    def smack_and_scoop(self):
        return
