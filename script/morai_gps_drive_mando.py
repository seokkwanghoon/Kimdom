#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from tf.transformations import euler_from_quaternion
import csv
import math
import matplotlib.pyplot as plt

import rospy
from morai_msgs.msg import CtrlCmd, Lamps
from morai_msgs.srv import MoraiEventCmdSrv
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

from Status import Carstatus
from PID import PID
from morai_sensor import MoraiOdom
from std_msgs.msg import Float32MultiArray
from yolo import TrafficLight
import os

username = os.environ.get('USER') or os.environ.get('USERNAME')
print(username)
class GpsDriver():

    def __init__(self) -> None:
        self.my_car = Carstatus()
        self.ctrl_cmd = CtrlCmd()
        self.odom = MoraiOdom()

        
        self.min_idx_pub = rospy.Publisher('min_idx', Int32, queue_size=1)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.car_status)
        
        self.pub_ctrl = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=1)
        self.srv_event_cmd = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)

        self.sub_tf_cls=rospy.Subscriber('tf_cls',Float32MultiArray,self.tf_cls_callback)
        
        #rospy.wait_for_message('obs_steer', Float64)
        rospy.wait_for_message('/odom',Odometry)


        self.manual_steering, self.manual_accel, self.manual_brake = 0, 0.5, 0
        self.theta = 0

        self.event_cmd_srv = MoraiEventCmdSrv()
        self.event_cmd_srv.ctrl_mode=3
        self.event_cmd_srv.gear=-1
        self.event_cmd_srv.set_pause=False

        self.ctrl_cmd.longlCmdType = 2
        self.event_cmd_srv.ctrl_mode=3
        self.event_cmd_srv.gear=-1
        self.event_cmd_srv.set_pause=False

        self.lamps = Lamps()
        self.gear = 1
        self.current_lamp = 0
        self.loop_hz = 50
        
        self.obs_steer = Float64()
        self.my_car.v = 15
        self.target_speed = 20/3.6
        self.accel = 0 
        self.stanley_k = 5
        self.minimum_idx = 0 
        self.slope_heading = 0
        self.stanley_error = 0
        self.x_list, self.y_list = [], []
        self.LOOK_AHEAD_DIST = 0.2
        self.pid_steer = PID(1.8,0,0,1/self.loop_hz,1)
        self.global_path(f'/home/{username}/catkin_ws/src/mando_morai/rep/mando_map_test.csv')
        self.obj=[]
    
    def obs_steer_callback(self, msg):
        self.obs_steer = msg.data

    def car_status(self,msg):
        self.my_car.x, self.my_car.y = msg.pose.pose.position.x, msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler_angles = euler_from_quaternion(quaternion)
        self.my_car.heading = self.nomarlize_pi(-euler_angles[2]+np.pi/2) # radian
        #print(self.my_car.heading)

    def tf_cls_callback(self, msg):
        self.obj = msg.data

        
            

    def drive_pub(self, angle: float):
        # rospy.loginfo(f'{self.manual_accel:0.1f}, {self.manual_brake:0.1f}, {self.manual_steering:0.1f}')
        # rospy.loginfo(f'speed: {speed:5.2f}m/s, angle: {angle:5.2f}, c_angle={angle:5.2f}, stanley: {self.stanley_error:2.3f}')
                
        #print(self.ctrl_cmd.accel, self.ctrl_cmd.brake)
        self.ctrl_cmd.steering = angle
        #rospy.loginfo(f"angle:{self.ctrl_cmd.steering:5.2f}, speed:{self.my_car.v*3.6:5.2f}")
        self.pub_ctrl.publish(self.ctrl_cmd)

    def global_path(self, file_name):
        with open(file_name, 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            next(csvreader, None)
            for row in csvreader:                
                self.x_list.append(float(row[0]))
                self.y_list.append(float(row[1]))
        self.len_list = len(self.x_list) - 1
        rospy.loginfo("-----Path loaded----------")
        # plt.figure()
        # plt.ion()
        # plt.show()

    def update_plot(self):
        plt.clf()
        plt.scatter(self.x_list, self.y_list,s=0.01,label='map', color='blue')

        plt.scatter(self.my_car.x, self.my_car.y, label='Current Position')
        arrow_length = 100
        arrow_end_x = self.my_car.x + arrow_length * np.cos(self.my_car.heading)
        arrow_end_y = self.my_car.y + arrow_length * np.sin(self.my_car.heading)
        plt.arrow(self.my_car.x, self.my_car.y, arrow_end_x-self.my_car.x, arrow_end_y-self.my_car.y, head_width = 3, head_length=1, fc='red', ec='red')
        plt.scatter(self.target_x, self.target_y, c='red', label='target Point')
        slope_x = self.target_x + (arrow_length + 0.2) * np.cos(self.slope_heading)
        slope_y = self.target_y + (arrow_length + 0.2) * np.sin(self.slope_heading)
        plt.arrow(self.target_x, self.target_y, slope_x - self.target_x, slope_y - self.target_y, head_width = 3, head_length = 2, fc='green', ec='green')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.title('Position Plot')
        plt.grid(True)
        plt.pause(0.01)

    #----- Path tracking -------#
    def calc_ahead_point(self):
        dx = self.LOOK_AHEAD_DIST * np.cos(self.my_car.heading)
        dy = self.LOOK_AHEAD_DIST * np.sin(self.my_car.heading)

        ahead_x = self.my_car.x + dx
        ahead_y = self.my_car.y + dy

        return ahead_x, ahead_y

    def calc_nearest_point(self, ahead_x, ahead_y):
        self.minimum_idx = 0
        minimum_dist = 1e7
        for i, (rx, ry) in enumerate(zip(self.x_list, self.y_list)):
            dist = math.dist((ahead_x, ahead_y), (rx, ry))            
            if (dist < minimum_dist):
                minimum_dist = dist
                self.minimum_idx = i
        first = (self.x_list[self.minimum_idx], self.y_list[self.minimum_idx])
        # rospy.loginfo(f'theta:{first[0]:5.2f}')
        self.min_idx_pub.publish(self.minimum_idx)
        rospy.loginfo(self.minimum_idx)
        return first
    #----- Path tracking -------#

    def nomarlize_pi(self, data):
        if data > np.pi:
            data = -2*np.pi + data
        elif data < -np.pi:
            data = 2*np.pi + data
        return data

    def stanley_control(self):
        self.target_x, self.target_y = self.calc_nearest_point(self.my_car.x, self.my_car.y)
        dx, dy = self.my_car.x - self.target_x , self.my_car.y - self.target_y
        #print(self.minimum_idx)
        self.Calc_slopeofpath()
        cte = - np.dot([dx, dy], [np.cos(self.my_car.heading + np.pi/2), np.sin(self.my_car.heading + np.pi/2)])
        cross_track_steering = np.arctan(self.stanley_k * cte / self.my_car.v + 1e-6)
        heading_error = self.nomarlize_pi(self.slope_heading - self.my_car.heading)
        self.stanley_error = self.nomarlize_pi(cross_track_steering + heading_error)
        steer = -self.pid_steer.do(self.stanley_error)  #if abs(cte) > 0.05 else 0
        #steer = 0.5 + self.stanley_error
        # rospy.loginfo((f'{self.my_car.v:5.2f},stanley_st:{steer:5.2f}, path:{self.slope_heading:5.2f}, myheading:{self.my_car.heading:5.2f}, h_err:{heading_error:5.2f},cte:{cte:5.2f}, cts:{cross_track_steering:5.2f} '))
       
        return steer
    
    # Calculate slope of current path
    def Calc_slopeofpath(self):
        idx_1 = self.minimum_idx
        if (self.minimum_idx + 1) > self.len_list:
            idx_2 = 0
        else:
            idx_2 = self.minimum_idx + 1
        
        x_1, y_1 = self.x_list[idx_1], self.y_list[idx_1]
        x_2, y_2 = self.x_list[idx_2], self.y_list[idx_2]

        dx = x_1 - x_2
        dy = y_1 - y_2

        self.slope_heading = self.nomarlize_pi(- np.arctan2(dx , dy) - np.pi/2)

    def speed_control(self):
        self.ctrl_cmd.velocity = 15

    def speed_limiter(self):
        self.target_speed = 15

    def run(self,show=False):    
        angle = self.stanley_control()

        self.speed_limiter()
        self.speed_control()
        self.drive_pub(angle)
        if show:
            self.update_plot()
    def stop(self):
        self.ctrl_cmd.steering=0.0
        self.ctrl_cmd.velocity=0
        self.pub_ctrl.publish(self.ctrl_cmd)
    

if __name__ == '__main__':
    rospy.init_node('morai_gps_drive')
    gps_drive = GpsDriver()
    yolo_recog=TrafficLight()
    loop_hz = 50
    rate = rospy.Rate(loop_hz)

    while not rospy.is_shutdown():
        camera_range=gps_drive.minimum_idx # gps => index
        camera=gps_drive.obj
        traffic_light=[5,12]
        flag = False
        for signal in traffic_light:
            if (signal in camera) and (3500<=camera_range<=3504):
                gps_drive.stop()
                flag = True
                break
            elif (signal in camera) and (2086<=camera_range<=2091):
                gps_drive.stop()
                flag=True
                break
            elif (signal in camera) and (6591<=camera_range<=6596):
                gps_drive.stop()
                flag=True
                break
        if flag==False:
            gps_drive.run(show=False)
    
        rate.sleep()