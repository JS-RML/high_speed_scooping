#!/usr/bin/python3
import time

import numpy as np
from numpy import deg2rad, rad2deg

# from odrive_msgs.msg import Odrive, OdriveStamped, Axis
# from ddh_gripper.msg import GripperCmd, FingerState, GripperState
# import rospy
import odrive
from odrive.enums import *
import yaml


def arm(axis, gain):
    axis.controller.config.input_mode = INPUT_MODE_POS_FILTER
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    axis.controller.config.pos_gain = gain
    axis.controller.config.vel_gain = 1
    axis.controller.config.input_filter_bandwidth = 100

def disarm(axis):
    axis.requested_state = AXIS_STATE_IDLE

def set_pos_gain(axis, pos_gain):
    axis.controller.config.pos_gain = pos_gain 


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


class DDGripper(object):

    def __init__(self):

        print('reading gripper parameters...')
        config_file = "ddh_default.yaml"
        with open("../config/"+config_file, 'r') as stream:
            try:
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

    def arm(self, gain = 250):
        arm(self.finger_L.axis0, gain)
        arm(self.finger_L.axis1, gain)
        arm(self.finger_R.axis0, gain)
        arm(self.finger_R.axis1, gain)

    def disarm(self):
        disarm(self.finger_L.axis0)
        disarm(self.finger_L.axis1)
        disarm(self.finger_R.axis0)
        disarm(self.finger_R.axis1)

    def set_stiffness(self,gain):
        set_pos_gain(self.finger_L.axis0, gain)
        set_pos_gain(self.finger_L.axis1, gain)
        set_pos_gain(self.finger_R.axis0, gain)
        set_pos_gain(self.finger_R.axis1, gain)

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

    @property
    def right_a1(self):
        return (self.link_pos_r0+self.link_pos_r1)/2

    @property
    def right_a2(self):
        return (self.link_pos_r1-self.link_pos_r0)/2

    @property
    def left_a1(self):
        return (self.link_pos_l0+self.link_pos_l1)/2

    @property
    def left_a2(self):
        return (self.link_pos_l0-self.link_pos_l1)/2

    # first control the position of individual links

    def set_link_pos_r0(self, link_pos):
        motor_pos = link_pos - self.R0_link
        motor_pos = motor_pos/360.
        motor_input = motor_pos * self.R0_dir + self.R0_offset
        self.finger_R.axis0.controller.input_pos = motor_input

    def set_link_pos_r1(self, link_pos):
        motor_pos = link_pos - self.R1_link
        motor_pos = motor_pos/360.
        motor_input = motor_pos * self.R1_dir + self.R1_offset
        self.finger_R.axis1.controller.input_pos = motor_input

    def set_link_pos_l0(self, link_pos):
        motor_pos = link_pos - self.L0_link
        motor_pos = motor_pos/360.
        motor_input = motor_pos * self.L0_dir + self.L0_offset
        self.finger_L.axis0.controller.input_pos = motor_input

    def set_link_pos_l1(self, link_pos):
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
        self.set_left_a1_a2(-90, 25)
        self.set_right_a1_a2(90, 25)
        time.sleep(0.5)
        self.set_parallel_jaw(-15, 0)
        time.sleep(0.5)
        self.set_parallel_jaw(25, 0)
        time.sleep(0.5)
        self.set_parallel_jaw(-15, 0)
        time.sleep(0.5)
        self.set_parallel_jaw(-10, 15)
        time.sleep(0.5)
        self.set_parallel_jaw(-10, -15)
        time.sleep(0.5)
        self.set_parallel_jaw(0, 0)


if __name__ == "__main__":
    # rospy.init_node('ddh_driver_node')
    gripper = DDGripper()
    gripper.arm()
    gripper.startup_dance()

    # rospy.spin()
