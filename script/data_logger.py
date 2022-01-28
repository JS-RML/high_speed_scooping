#!/usr/bin/python3
import os
import time
import json
import threading
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import numpy as np



class DataLogger:

    def __init__(self, robot, gripper, scooping_primitives):
        self.ur = robot
        self.ddh = gripper
        self.scoop = scooping_primitives

        self.keep_logging = False
        self.log_rate1 = 300
        self.log_rate2 = 100
        self.log_rate3 = 200
        self.logging_thread1 = None
        self.logging_thread2 = None
        self.logging_thread3 = None

        self.logged_data1 = []
        self.logged_data2 = []
        self.logged_data3 = []
        self.collision_time = None

    def logging_job1(self):
        while self.keep_logging:
            time.sleep(1.0/self.log_rate1)
            data = {
                't': round(time.time() * 1000),
                'L_stiff': self.ddh.cmd_stiff_l,
                'R_stiff': self.ddh.cmd_stiff_r,
                'L0': self.ddh.link_pos_l0,
                'L1': self.ddh.link_pos_l1,
                'R0': self.ddh.link_pos_r0,
                'R1': self.ddh.link_pos_r1,
                'L0_cmd': self.ddh.cmd_pos_l0,
                'L1_cmd': self.ddh.cmd_pos_l1,
                'R0_cmd': self.ddh.cmd_pos_r0,
                'R1_cmd': self.ddh.cmd_pos_r1,
                # 'psi': self.ddh.right_phi + self.scoop.theta, # cause delay in log
            }
            self.logged_data1.append(data)
            if self.scoop.collided:
                self.collision_time = data['t']
                self.scoop.collided = False
        
    def logging_job2(self):
        while self.keep_logging:
            time.sleep(1.0/self.log_rate2)
            data = {
                't2': round(time.time() * 1000),
                'UR_Z': self.ur.getl(wait=True,_log=False)[2],
                'UR_Z_SPD': self.ur.get_tcp_speed(wait=False)[2]
            }
            self.logged_data2.append(data)

    def logging_job3(self):
        while self.keep_logging:
            time.sleep(1.0/self.log_rate3)
            data = {
                't3': round(time.time() * 1000),
                'psi': self.ddh.right_phi + self.scoop.theta
            }
            self.logged_data3.append(data)

    @property
    def logged(self):
        return self.logging_thread1 is not None

    @logged.setter
    def logged(self, log):
        if log and not self.logged:
            self.log_start()
        if not log and self.logged:
            self.log_stop()
    
    def log_start(self):
        self.logged_data1.clear()
        self.logged_data2.clear()
        self.logged_data3.clear()
        self.keep_logging = True
        self.logging_thread1 = threading.Thread(target=self.logging_job1)
        self.logging_thread1.start()
        self.logging_thread2 = threading.Thread(target=self.logging_job2)
        self.logging_thread2.start()
        self.logging_thread3 = threading.Thread(target=self.logging_job3)
        self.logging_thread3.start()
        print('Start logging')

    def log_stop(self):
        self.keep_logging = False
        self.logging_thread1.join()
        self.logging_thread1 = None
        self.logging_thread2.join()
        self.logging_thread2 = None
        self.logging_thread3.join()
        self.logging_thread3 = None
        print('Stop logging')

    def plot_data(self, save_plot = False):
        plot_item = {}
        for i in self.logged_data1[0]:
            plot_item[i] = []
        for j in self.logged_data2[0]:
            plot_item[j] = []
        for k in self.logged_data3[0]:
            plot_item[k] = []

        # convert logged_data(list of dict) to plot_item(dict of list)
        for d in self.logged_data1:
            for name in d:
                if name == 't': plot_item[name].append(d[name]-self.logged_data1[0][name])
                else: plot_item[name].append(d[name])

        for d in self.logged_data2:
            for name in d:
                if name == 't2': plot_item[name].append(d[name]-self.logged_data2[0][name])
                else: plot_item[name].append(d[name])

        for d in self.logged_data3:
            for name in d:
                if name == 't3': plot_item[name].append(d[name]-self.logged_data3[0][name])
                else: plot_item[name].append(d[name])

        if self.collision_time is not None:
            self.collision_time = self.collision_time - self.logged_data1[0]['t']
            plot_item['t_col'] = [self.collision_time]
            print("Collision time: {} ms".format(self.collision_time))
        
        # creat subsplots
        fig1, ax1 = plt.subplots(1,2)
        fig2, ax2 = plt.subplots(1,2)
        fig3, ax3 = plt.subplots(1,2)

        # plot motor angle
        ax1[0].plot(plot_item['t'], plot_item['L0'], label='F0', color='tab:blue')
        ax1[0].plot(plot_item['t'], plot_item['L1'], label='F1', color='tab:orange')
        ax1[0].plot(plot_item['t'], plot_item['R0'], label='T0', color='tab:green')
        ax1[0].plot(plot_item['t'], plot_item['R1'], label='T1', color='tab:red')
        ax1[0].plot(plot_item['t'], plot_item['L0_cmd'], color='tab:blue', linestyle='--')
        ax1[0].plot(plot_item['t'], plot_item['L1_cmd'], color='tab:orange', linestyle='--')
        ax1[0].plot(plot_item['t'], plot_item['R0_cmd'], color='tab:green', linestyle='--')
        ax1[0].plot(plot_item['t'], plot_item['R1_cmd'], color='tab:red', linestyle='--')
        if self.collision_time is not None: ax1[0].axvline(x=plot_item['t_col'], color='darkgray', linewidth='1.5', linestyle='--')
        ax1[0].legend(loc='upper right').get_frame().set_linewidth(1.0)
        ax1[0].set_title("Readings of motor angles")
        ax1[0].set_ylabel("Angle (degree)")
        ax1[0].set_xlabel("Time (ms)")
        ax1[0].grid(True)

        # plot angle of attack
        ax1[1].plot(plot_item['t3'], plot_item['psi'])
        if self.collision_time is not None: ax1[1].axvline(x=plot_item['t_col'], color='darkgray', linewidth='1.5' ,linestyle='--')
        ax1[1].set_title("Readings of psi (angle of attack)")
        ax1[1].set_ylabel("Angle (degree)")
        ax1[1].set_xlabel("Time (ms)")
        ax1[1].grid(True)

        # plot stiffness
        ax2[0].plot(plot_item['t'], plot_item['L_stiff'], label='F_Kp')
        ax2[0].plot(plot_item['t'], plot_item['R_stiff'], label='T_Kp')
        if self.collision_time is not None: ax2[0].axvline(x=plot_item['t_col'], color='darkgray', linewidth='1.5' ,linestyle='--')
        ax2[0].legend(loc='lower right').get_frame().set_linewidth(1.0)
        ax2[0].set_title("Stiffness of fingers (proportional gain of position controller)")
        ax2[0].set_ylabel("Gain ((turn/s) / turn)")
        ax2[0].set_xlabel("Time (ms)")
        ax2[0].grid(True)

        # plot ur z
        ur_z_interp = interp1d(plot_item['t2'],plot_item['UR_Z'],kind="cubic")
        t_sample = np.linspace(plot_item['t2'][0],plot_item['t2'][-1],100)
        ur_z_new = ur_z_interp(t_sample)
        ax3[0].plot(t_sample, ur_z_new)
        # ax3[0].plot(plot_item['t2'], plot_item['UR_Z'])
        if self.collision_time is not None: ax3[0].axvline(x=plot_item['t_col'], color='darkgray', linewidth='1.5' ,linestyle='--')
        ax3[0].set_title("Height of arm")
        ax3[0].set_ylabel("Height (m)")
        ax3[0].set_xlabel("Time (ms)")

        # plot ur z spd
        ax3[1].plot(plot_item['t2'], plot_item['UR_Z_SPD'])
        if self.collision_time is not None: ax3[1].axvline(x=plot_item['t_col'], color='darkgray', linewidth='1.5' ,linestyle='--')
        ax3[1].set_title("Speed of arm")
        ax3[1].set_ylabel("Speed (m/s)")
        ax3[1].set_xlabel("Time (ms)")

        if save_plot: 
            currentTime = time.strftime("%Y-%m-%d_%H:%M:%S", time.localtime())
            save_dir = 'plots/' + str(currentTime)
            os.makedirs(save_dir)

            # save two subplots as two figs
            fig1sub1 = ax1[0].get_window_extent().transformed(fig1.dpi_scale_trans.inverted())
            fig1sub2 = ax1[1].get_window_extent().transformed(fig1.dpi_scale_trans.inverted())
            fig1.savefig(save_dir + '/joint.png', bbox_inches=fig1sub1.expanded(1.23, 1.23))
            fig1.savefig(save_dir + '/psi.png', bbox_inches=fig1sub2.expanded(1.23, 1.23))
            fig2sub1 = ax2[0].get_window_extent().transformed(fig2.dpi_scale_trans.inverted())
            fig2.savefig(save_dir + '/gain.png', bbox_inches=fig2sub1.expanded(1.23, 1.23))
            fig3sub1 = ax3[0].get_window_extent().transformed(fig3.dpi_scale_trans.inverted())
            fig3sub2 = ax3[1].get_window_extent().transformed(fig3.dpi_scale_trans.inverted())
            fig3.savefig(save_dir + '/height.png', bbox_inches=fig3sub1.expanded(1.23, 1.23))
            fig3.savefig(save_dir + '/spd.png', bbox_inches=fig3sub2.expanded(1.23, 1.23))
            # save logged data
            with open(save_dir + '/data.json', mode='w') as f:
                json.dump(plot_item, f)

        plt.show()
        self.collision_time = None