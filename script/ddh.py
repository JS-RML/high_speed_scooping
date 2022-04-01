#!/usr/bin/python3

# Note: This python script is used for temporary testing only.
# Please use the latest driver in https://github.com/HKUST-RML/ddh_driver

import time

import numpy as np
from numpy import deg2rad, rad2deg

# from odrive_msgs.msg import Odrive, OdriveStamped, Axis
# from ddh_gripper.msg import GripperCmd, FingerState, GripperState
# import rospy
import odrive
from odrive.enums import *
import yaml

def arm(axis):
    axis.controller.config.input_mode = INPUT_MODE_POS_FILTER #INPUT_MODE_PASSTHROUGH
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

def disarm(axis):
    axis.requested_state = AXIS_STATE_IDLE

def set_pos_gain(axis, pos_gain):
    axis.controller.config.pos_gain = pos_gain 

def set_vel_gain(axis, vel_gain):
    axis.controller.config.vel_gain = vel_gain 

def set_input_bandwidth(axis, BW):
    axis.controller.config.input_filter_bandwidth = BW

# def axis2ros(ax):
#     msg = Axis()
#     msg.pos_estimate = ax.encoder.pos_estimate
#     msg.vel_estimate = ax.encoder.vel_estimate
#     msg.input_pos = ax.controller.input_pos
#     msg.input_vel = ax.controller.input_vel
#     msg.input_torque = ax.controller.input_torque
#     msg.setpoint_pos = ax.controller.pos_setpoint
#     msg.setpoint_vel = ax.controller.vel_setpoint
#     msg.setpoint_torque = ax.controller.torque_setpoint
#     msg.current_meas_phB = ax.motor.current_meas_phB
#     msg.current_meas_phC = ax.motor.current_meas_phC
#     msg.fet_thermistor_temperature = ax.fet_thermistor.temperature
#     return msg


# def odrive2ros(od):
#     msg = Odrive()
#     msg.axis0 = axis2ros(od.axis0)
#     msg.axis1 = axis2ros(od.axis1)
#     return msg


class Gripper(object):

    def __init__(self, config_name):
        config_file = config_name + ".yaml"
        with open("../config/"+config_file, 'r') as stream:
            try:
                print('reading gripper config...')
                config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        # self.R0_offset = rospy.get_param('/motors/R0/offset')
        # self.R0_dir = rospy.get_param('/motors/R0/dir')
        # self.R1_offset = rospy.get_param('/motors/R1/offset')
        # self.R1_dir = rospy.get_param('/motors/R1/dir')
        # self.L0_offset = rospy.get_param('/motors/L0/offset')
        # self.L0_dir = rospy.get_param('/motors/L0/dir')
        # self.L1_offset = rospy.get_param('/motors/L1/offset')
        # self.L1_dir = rospy.get_param('/motors/L1/dir')
        # self.R0_link = rospy.get_param('/linkages/R0')
        # self.R1_link = rospy.get_param('/linkages/R1')
        # self.L0_link = rospy.get_param('/linkages/L0')
        # self.L1_link = rospy.get_param('/linkages/L1')
        # self.geometry_l1 = rospy.get_param('/geometry/l1')
        # self.geometry_l2 = rospy.get_param('/geometry/l2')
        # self.geometry_beta = rospy.get_param('/geometry/beta')
        self.R0_offset = config['motors']['R0']['offset']
        self.R0_dir = config['motors']['R0']['dir']
        self.R1_offset = config['motors']['R1']['offset']
        self.R1_dir = config['motors']['R1']['dir']
        self.L0_offset = config['motors']['L0']['offset']
        self.L0_dir = config['motors']['L0']['dir']
        self.L1_offset =config['motors']['L1']['offset']
        self.L1_dir = config['motors']['L1']['dir']
        self.R0_link = config['linkages']['R0']
        self.R1_link = config['linkages']['R1']
        self.L0_link = config['linkages']['L0']
        self.L1_link = config['linkages']['L1']
        self.geometry_l1 = config['geometry']['l1']
        self.geometry_l2 = config['geometry']['l2']
        self.geometry_beta = config['geometry']['beta']
        self.geometry_l3 = config['geometry']['l3']
        self.geometry_gamma = config['geometry']['gamma']
        self.a2_sing = rad2deg(np.arcsin(self.geometry_l2/self.geometry_l1)) # a2 when distal links in singularity
        # virtual link formed by l2 and l3
        self._l3 = np.sqrt(self.geometry_l2**2 + self.geometry_l3**2 - 2 * self.geometry_l2 * self.geometry_l3 * np.cos(deg2rad(self.geometry_gamma)))
        # angle between l2 and _l3
        self._gamma = rad2deg(np.arcsin(np.sin(deg2rad(self.geometry_gamma))/self._l3*self.geometry_l3))
        # range of IK for the distal joint
        self.r_min = np.sqrt(self.geometry_l1**2 - self.geometry_l2**2) + config['geometry']['r_min_offset']
        self.r_max = self.geometry_l1 + self.geometry_l2 - config['geometry']['r_max_offset']

        print('connecting to odrive...')
        self.finger_L = odrive.find_any(serial_number='207E39775453')
        print('found left finger')
        self.finger_R = odrive.find_any(serial_number='207C39785453')
        print('found right fingers')
        # arm(self.finger_L.axis0)
        # arm(self.finger_L.axis1)
        # arm(self.finger_R.axis0)
        # arm(self.finger_R.axis1)
        # self.pub_odrive_L = rospy.Publisher('/ddh/odrive/L', OdriveStamped, queue_size=10)
        # self.pub_odrive_R = rospy.Publisher('/ddh/odrive/R', OdriveStamped, queue_size=10)
        # self.pub_gripper_state = rospy.Publisher('/ddh/state', GripperState, queue_size=10)
        # self.timer = rospy.Timer(rospy.Duration(0.02), self.refresh)

        # commanded link angle
        self.cmd_pos_l0 = None
        self.cmd_pos_l1 = None
        self.cmd_pos_r0 = None
        self.cmd_pos_r1 = None
        # commanded stiffness
        self.cmd_stiff_l = None
        self.cmd_stiff_r = None

    def arm(self, pos_gain = 250, vel_gain = 1, BW = 500, finger = 'LR'):
        set_axis = []
        if finger == 'LR':
            set_axis = [self.finger_L.axis0,self.finger_L.axis1,self.finger_R.axis0,self.finger_R.axis1]
        elif finger == 'L':
            set_axis = [self.finger_L.axis0,self.finger_L.axis1]
        elif finger == 'R':
            set_axis = [self.finger_R.axis0,self.finger_R.axis1]
        else:
            print("Invalid finger argument.")
            return
        self.set_stiffness(pos_gain, finger)
        self.set_vel_gain(vel_gain, finger)
        self.set_bandwidth(BW, finger)
        for ax in set_axis:
            arm(ax)

    def disarm(self, finger = 'LR'):
        set_axis = []
        if finger == 'LR':
            set_axis = [self.finger_L.axis0,self.finger_L.axis1,self.finger_R.axis0,self.finger_R.axis1]
        elif finger == 'L':
            set_axis = [self.finger_L.axis0,self.finger_L.axis1]
        elif finger == 'R':
            set_axis = [self.finger_R.axis0,self.finger_R.axis1]
        else:
            print("Invalid finger argument.")
            return
        for ax in set_axis:
            disarm(ax)

    def set_stiffness(self, gain, finger = 'LR'):
        set_axis = []
        if finger == 'LR':
            self.cmd_stiff_l = gain
            self.cmd_stiff_r = gain
            set_axis = [self.finger_L.axis0,self.finger_L.axis1,self.finger_R.axis0,self.finger_R.axis1]
        elif finger == 'L':
            self.cmd_stiff_l = gain
            set_axis = [self.finger_L.axis0,self.finger_L.axis1]
        elif finger == 'R':
            self.cmd_stiff_r = gain
            set_axis = [self.finger_R.axis0,self.finger_R.axis1]
        else:
            print("Invalid finger argument.")
            return
        for ax in set_axis:
            set_pos_gain(ax, gain)

    def set_vel_gain(self, gain, finger = 'LR'):
        set_axis = []
        if finger == 'LR':
            set_axis = [self.finger_L.axis0,self.finger_L.axis1,self.finger_R.axis0,self.finger_R.axis1]
        elif finger == 'L':
            set_axis = [self.finger_L.axis0,self.finger_L.axis1]
        elif finger == 'R':
            set_axis = [self.finger_R.axis0,self.finger_R.axis1]
        else:
            print("Invalid finger argument.")
            return
        for ax in set_axis:
            set_vel_gain(ax, gain)

    def set_bandwidth(self, BW, finger = 'LR'):
        set_axis = []
        if finger == 'LR':
            set_axis = [self.finger_L.axis0,self.finger_L.axis1,self.finger_R.axis0,self.finger_R.axis1]
        elif finger == 'L':
            set_axis = [self.finger_L.axis0,self.finger_L.axis1]
        elif finger == 'R':
            set_axis = [self.finger_R.axis0,self.finger_R.axis1]
        else:
            print("Invalid finger argument.")
            return
        for ax in set_axis:
            set_input_bandwidth(ax, BW)

    @property
    def motor_pos_r0(self):
        return 360 * self.R0_dir * (self.finger_R.axis0.encoder.pos_estimate - self.R0_offset)

    @property
    def motor_pos_r1(self):
        return 360 * self.R1_dir * (self.finger_R.axis1.encoder.pos_estimate - self.R1_offset)

    @property
    def motor_pos_l0(self):
        return 360 * self.L0_dir * (self.finger_L.axis0.encoder.pos_estimate - self.L0_offset)

    @property
    def motor_pos_l1(self):
        return 360 * self.L1_dir * (self.finger_L.axis1.encoder.pos_estimate - self.L1_offset)

    @property
    def link_pos_r0(self):
        return self.motor_pos_r0 + self.R0_link

    @property
    def link_pos_r1(self):
        return self.motor_pos_r1 + self.R1_link

    @property
    def link_pos_l0(self):
        return self.motor_pos_l0 + self.L0_link

    @property
    def link_pos_l1(self):
        return self.motor_pos_l1 + self.L1_link

    # forward kinematics function: link angles to a1, a2 angle 
    def link_to_a1(self, l0, l1):
        return (l0+l1)/2

    def link_to_a2(self, l0, l1):
        return np.absolute(l0-l1)/2

    @property
    def right_a1(self):
        return self.link_to_a1(self.link_pos_r0, self.link_pos_r1)
        # return (self.link_pos_r0+self.link_pos_r1)/2

    @property
    def right_a2(self):
        return self.link_to_a2(self.link_pos_r0, self.link_pos_r1)
        # return (self.link_pos_r1-self.link_pos_r0)/2

    @property
    def left_a1(self):
        # return (self.link_pos_l0+self.link_pos_l1)/2
        return self.link_to_a1(self.link_pos_l0, self.link_pos_l1)

    @property
    def left_a2(self):
        return self.link_to_a2(self.link_pos_l0, self.link_pos_l1)
        # return (self.link_pos_l0-self.link_pos_l1)/2

    # r: distance from motor joint to distal joint (base joint of finger)
    # forward kinematics function: a2 angle to r distance
    def a2_to_r(self, a2):
        if a2 > self.a2_sing:
            return  self.geometry_l1*np.cos(deg2rad(a2))
        else: 
            return self.geometry_l1*np.cos(deg2rad(a2)) + np.sqrt(self.geometry_l2**2 - (self.geometry_l1*np.sin(deg2rad(a2)))**2)

    @property
    def left_finger_dist(self):
        # distal_r = np.sqrt(self.geometry_l2**2 - (self.geometry_l1*np.sin(deg2rad(self.left_a2)))**2)
        # if np.isnan(distal_r):
        #     distal_r = 0
        # return self.geometry_l1*np.cos(deg2rad(self.left_a2)) + distal_r
        return self.a2_to_r(self.left_a2)

    @property
    def right_finger_dist(self):
        # distal_r = np.sqrt(self.geometry_l2**2 - (self.geometry_l1*np.sin(deg2rad(self.right_a2)))**2)
        # if np.isnan(distal_r):
        #     distal_r = 0
        # return self.geometry_l1*np.cos(deg2rad(self.right_a2)) + distal_r
        return self.a2_to_r(self.right_a2)

    # forward kinematics function: r, a2 angle to distal joint coordinate
    def r_a1_to_rx_ry(self, r, a1):
        # rx, ry
        return r * np.cos(deg2rad(a1)), r * np.sin(deg2rad(a1))

    # position of distal joint (base joint of finger) in motor frame
    @property
    def left_finger_pos(self): 
        # x = self.left_finger_dist * np.cos(deg2rad(self.left_a1))
        # y = self.left_finger_dist * np.sin(deg2rad(self.left_a1))
        # return x, y
        return self.r_a1_to_rx_ry(self.left_finger_dist, self.left_a1)

    @property
    def right_finger_pos(self): 
        # x = self.right_finger_dist * np.cos(deg2rad(self.right_a1))
        # y = self.right_finger_dist * np.sin(deg2rad(self.right_a1))
        # return x, y
        return self.r_a1_to_rx_ry(self.right_finger_dist, self.right_a1)

    # a3: angle between distal link (L2) and vector from origin to distal joint (r)
    # forward kinematics function: r distance to a3 angle
    def r_to_a3(self, r):
        return rad2deg(np.arccos((self.geometry_l1**2 - self.geometry_l2**2 - r**2)/(-2 * self.geometry_l2 * r)))

    @property
    def left_a3(self):
        # return rad2deg(np.arccos((self.geometry_l1**2 - self.geometry_l2**2 - self.left_finger_dist**2)/(-2 * self.geometry_l2 * self.left_finger_dist)))
        return self.r_to_a3(self.left_finger_dist)

    @property
    def right_a3(self):
        # return rad2deg(np.arccos((self.geometry_l1**2 - self.geometry_l2**2 - self.right_finger_dist**2)/(-2 * self.geometry_l2 * self.right_finger_dist)))
        return self.r_to_a3(self.right_finger_dist)

    # phi: angle of finger surface relative to x axis
    # forward kinematics function: link angles to phi angle
    def link_to_phi(self, l0, l1, finger):
        a1 = self.link_to_a1(l0,l1)
        r = self.a2_to_r(self.link_to_a2(l0,l1))
        a3 = self.r_to_a3(r)
        if finger == 'L':
            return self.a1a3_to_L_phi(a1,a3)
        elif finger == 'R':
            return self.a1a3_to_R_phi(a1,a3)

    # forward kinematics function: a1, a3 angles to finger phi angle
    def a1a3_to_L_phi(self, a1, a3):
        return a1 + a3 + self.geometry_beta - 180
    def a1a3_to_R_phi(self, a1, a3):
        return a1 - (a3 + self.geometry_beta - 180)

    @property
    def left_phi(self):
        # return self.left_a1 + self.left_a3 + self.geometry_beta - 180
        return self.a1a3_to_L_phi(self.left_a1, self.left_a3)

    @property
    def right_phi(self):
        # return self.right_a1 - (self.right_a3 + self.geometry_beta - 180)
        return self.a1a3_to_R_phi(self.right_a1, self.right_a3)

    # position of fingertip in motor frame
    # forward kinematics function: link angles to tip coordinate
    def link_to_tip(self, l0, l1, finger):
        a1 = self.link_to_a1(l0,l1)
        r = self.a2_to_r(self.link_to_a2(l0,l1))
        a3 = self.r_to_a3(r)
        rxry = self.r_a1_to_rx_ry(r, a1)
        if finger == 'L':
            return self.get_L_tip(a1, a3, rxry)
        elif finger == 'R':
            return self.get_R_tip(a1,a3,rxry)

    # forward kinematics function: a1, a3 angles and joint coordinate to tip coordinate
    def a1a3rxy_to_tip(self, a1, a3, rxry, finger):
        # angle of l3 relative to x axis
        if finger == 'L':
            q_tip = a1 + a3 + self.geometry_gamma - 180
        elif finger == 'R':
            q_tip = a1 - (a3 + self.geometry_gamma - 180)
        x = rxry[0] + self.geometry_l3 * np.cos(deg2rad(q_tip))
        y = rxry[1] + self.geometry_l3 * np.sin(deg2rad(q_tip))
        return x, y

    @property
    def left_tip_pos(self):
        # # angle of l3 relative to x axis
        # q_tip = self.left_a1 + self.left_a3 + self.geometry_gamma - 180
        # x = self.left_finger_pos[0] + self.geometry_l3 * np.cos(deg2rad(q_tip))
        # y = self.left_finger_pos[1] + self.geometry_l3 * np.sin(deg2rad(q_tip))
        # return x, y
        return self.a1a3rxy_to_tip(self.left_a1, self.left_a3, self.left_finger_pos, 'L')

    @property
    def right_tip_pos(self):
        # # angle of l3 relative to x axis
        # q_tip = self.right_a1 - (self.right_a3 + self.geometry_gamma - 180)
        # x = self.right_finger_pos[0] + self.geometry_l3 * np.cos(deg2rad(q_tip))
        # y = self.right_finger_pos[1] + self.geometry_l3 * np.sin(deg2rad(q_tip))
        # return x, y
        return self.a1a3rxy_to_tip(self.right_a1, self.right_a3, self.right_finger_pos, 'R')

    # first control the position of individual links

    def set_link_pos_r0(self, link_pos):
        self.cmd_pos_r0 = link_pos
        motor_pos = link_pos - self.R0_link
        motor_pos = motor_pos/360.
        motor_input = motor_pos * self.R0_dir + self.R0_offset
        self.finger_R.axis0.controller.input_pos = motor_input

    def set_link_pos_r1(self, link_pos):
        self.cmd_pos_r1 = link_pos
        motor_pos = link_pos - self.R1_link
        motor_pos = motor_pos/360.
        motor_input = motor_pos * self.R1_dir + self.R1_offset
        self.finger_R.axis1.controller.input_pos = motor_input

    def set_link_pos_l0(self, link_pos):
        self.cmd_pos_l0 = link_pos
        motor_pos = link_pos - self.L0_link
        motor_pos = motor_pos/360.
        motor_input = motor_pos * self.L0_dir + self.L0_offset
        self.finger_L.axis0.controller.input_pos = motor_input

    def set_link_pos_l1(self, link_pos):
        self.cmd_pos_l1 = link_pos
        motor_pos = link_pos - self.L1_link
        motor_pos = motor_pos/360.
        motor_input = motor_pos * self.L1_dir + self.L1_offset
        self.finger_L.axis1.controller.input_pos = motor_input

    # then control the finger (a1,a2)

    def set_right_a1(self, a1):
        a2 = self.right_a2
        self.set_link_pos_r0(a1-a2)
        self.set_link_pos_r1(a1+a2)

    def set_right_a2(self, a2):
        a1 = self.right_a1
        self.set_link_pos_r0(a1-a2)
        self.set_link_pos_r1(a1+a2)

    def set_right_a1_a2(self, a1, a2):
        self.set_link_pos_r0(a1-a2)
        self.set_link_pos_r1(a1+a2)

    def set_left_a1(self, a1):
        a2 = self.left_a2
        self.set_link_pos_l0(a1+a2)
        self.set_link_pos_l1(a1-a2)

    def set_left_a2(self, a2):
        a1 = self.left_a1
        self.set_link_pos_l0(a1+a2)
        self.set_link_pos_l1(a1-a2)

    def set_left_a1_a2(self, a1, a2):
        self.set_link_pos_l0(a1+a2)
        self.set_link_pos_l1(a1-a2)

    # then add inverse kinematics

    def ik_right_a1_phi(self, a1, phi):
        a2_rad = np.arcsin(np.sin(-deg2rad(a1)+deg2rad(self.geometry_beta)+deg2rad(phi))*self.geometry_l2/self.geometry_l1)
        return a1, rad2deg(a2_rad)

    def set_right_a1_phi(self, a1, phi):
        cmd_a1, cmd_a2 = self.ik_right_a1_phi(a1, phi)
        self.set_right_a1_a2(cmd_a1, cmd_a2)

    def ik_left_a1_phi(self, a1, phi):
        a2_rad = np.arcsin(np.sin(deg2rad(a1)+deg2rad(self.geometry_beta)-deg2rad(phi))*self.geometry_l2/self.geometry_l1)
        return a1, rad2deg(a2_rad)

    def set_left_a1_phi(self, a1, phi):
        cmd_a1, cmd_a2 = self.ik_left_a1_phi(a1, phi)
        self.set_left_a1_a2(cmd_a1, cmd_a2)

    # ik of finger joint pos

    def ik_finger_pos(self, pos):
        x, y = pos
        r = np.sqrt(x**2+y**2)
        r = max(self.r_min,min(r, self.r_max))
        a1 = rad2deg(np.arctan2(y,x))
        a2 = rad2deg(np.arccos((self.geometry_l2**2-self.geometry_l1**2-r**2)/(-2*self.geometry_l1*r)))
        return a1, a2

    def set_left_finger_pos(self, pos):
        cmd_a1, cmd_a2 = self.ik_finger_pos(pos)
        self.set_left_a1_a2(cmd_a1, cmd_a2)

    def set_right_finger_pos(self, pos):
        cmd_a1, cmd_a2 = self.ik_finger_pos(pos)
        self.set_right_a1_a2(cmd_a1, cmd_a2)

    # ik of fingertip

    def ik_finger_tip(self, pos, finger):
        np.seterr(all = "raise")
        x_tip, y_tip = pos
        # link from origin to tip
        l_tip = np.sqrt(x_tip**2+y_tip**2)
        # angle of l_tip relative to x axis
        q_tip = rad2deg(np.arctan2(y_tip,x_tip))
        # angle between l1 and l_tip
        q_1_tip = rad2deg(np.arccos((self._l3**2 - self.geometry_l1**2 - l_tip**2)/(-2 * self.geometry_l1 * l_tip)))
        # angle of l1 relative to x axis
        if finger == 'L': # left finger
            q1 =  q_tip - q_1_tip
        elif finger == 'R': # right finger
            q1 = q_tip + q_1_tip
        # angle between l1 and _l3
        q_1__3 = rad2deg(np.arccos((l_tip**2 - self.geometry_l1**2 - self._l3**2)/(-2 * self.geometry_l1 * self._l3)))
        # angle between l1 and l2
        q21 = q_1__3 - self._gamma
        # angle of l2 relative to x axis
        if finger == 'L': # left finger
            q2 = 180 - q21 + q1
        elif finger == 'R': # right finger
            q2 = -180 + q21 + q1
        # target position of distal joint
        x = self.geometry_l1 * np.cos(deg2rad(q1)) + self.geometry_l2 * np.cos(deg2rad(q2))
        y = self.geometry_l1 * np.sin(deg2rad(q1)) + self.geometry_l2 * np.sin(deg2rad(q2))
        return self.ik_finger_pos((x,y))

    def set_left_tip(self, pos):
        # print("Setting left tip:", pos)
        try:
            cmd_a1, cmd_a2 = self.ik_finger_tip(pos, 'L')
        except:
            print('Target position out of finger workspace!')
            return
        self.set_left_a1_a2(cmd_a1, cmd_a2)

    def set_right_tip(self, pos):
        # print("Setting right tip:", pos)
        try:
            cmd_a1, cmd_a2 = self.ik_finger_tip(pos, 'R')
        except:
            print('Target position out of finger workspace!')
            return
        self.set_right_a1_a2(cmd_a1, cmd_a2)

    # parallel jaw

    def set_parallel_jaw(self, angle, phi):
        self.set_left_a1_phi(-angle, phi)
        self.set_right_a1_phi(angle, phi)

    # def publish_gripper_state(self):
    #     msg = GripperState()
    #     msg.header.stamp = rospy.Time.now()
    #     msg.header.frame_id = 'world'
    #     # motor pos
    #     msg.right.motor_pos_0 = self.motor_pos_r0
    #     msg.right.motor_pos_1 = self.motor_pos_r1
    #     msg.left.motor_pos_0 = self.motor_pos_l0
    #     msg.left.motor_pos_1 = self.motor_pos_l1
    #     # link pos
    #     msg.right.link_pos_0 = self.link_pos_r0
    #     msg.right.link_pos_1 = self.link_pos_r1
    #     msg.left.link_pos_0 = self.link_pos_l0
    #     msg.left.link_pos_1 = self.link_pos_l1
    #     self.pub_gripper_state.publish(msg)

    # def refresh(self, t):
    #     msg_r = OdriveStamped()
    #     msg_r.header.stamp = rospy.Time.now()
    #     msg_r.odrive = odrive2ros(self.finger_R)
    #     self.pub_odrive_R.publish(msg_r)
    #     msg_l = OdriveStamped()
    #     msg_l.header.stamp = rospy.Time.now()
    #     msg_l.odrive = odrive2ros(self.finger_L)
    #     self.pub_odrive_L.publish(msg_l)
    #     self.publish_gripper_state()

    def startup_dance(self):
        # defaut linakge
        # self.set_left_a1_a2(-90, 25)
        # self.set_right_a1_a2(90, 25)
        # time.sleep(0.5)
        # self.set_parallel_jaw(-15, 0)
        # time.sleep(0.5)
        # self.set_parallel_jaw(25, 0)
        # time.sleep(0.5)
        # self.set_parallel_jaw(-15, 0)
        # time.sleep(0.5)
        # self.set_parallel_jaw(-10, 15)
        # time.sleep(0.5)
        # self.set_parallel_jaw(-10, -15)
        # time.sleep(0.5)
        # self.set_parallel_jaw(0, 0)
        # scooping linkage
        self.set_left_a1_a2(-90, 25)
        self.set_right_a1_a2(90, 25)
        time.sleep(0.5)
        self.set_parallel_jaw(-10, 0)
        time.sleep(0.5)
        self.set_parallel_jaw(25, 0)
        time.sleep(0.5)
        self.set_parallel_jaw(-10, 0)
        time.sleep(0.5)
        self.set_parallel_jaw(-5, 15)
        time.sleep(0.5)
        self.set_parallel_jaw(-5, -15)
        time.sleep(0.5)
        self.set_parallel_jaw(0, 0)


if __name__ == "__main__":
    # rospy.init_node('ddh_driver_node')
    gripper = Gripper("ddh_scooping")
    # gripper.arm()
    # gripper.startup_dance()
    # gripper.set_left_tip((150,0))
    # gripper.set_right_tip((150,0))
    # gripper.set_left_tip((157, 40))
    # gripper.set_right_tip((157, -40))
    # while 1:
    #     print("=========================")
    #     print(gripper.left_phi,gripper.link_to_phi(gripper.link_pos_l0, gripper.link_pos_l1, 'L'))
        # print(gripper.left_tip_pos,gripper.right_tip_pos)
        # print((gripper.left_a2 + gripper.right_a2)/2)
        # print(gripper.right_a3)
        # time.sleep(0.2)

    # rospy.spin()
