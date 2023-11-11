#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from morai_sensor import MoraiCamera
import cv2
import rospy
from ultralytics import YOLO
import numpy as np
from std_msgs.msg import Float32MultiArray
import os

username = os.environ.get('USER') or os.environ.get('USERNAME')

class TrafficLight():
    def __init__(self) -> None:
        self.model = YOLO(f'/home/{username}/catkin_ws/src/Kimdom/rep/best.pt')  # model 입력
        self.cls_pub = rospy.Publisher('/tf_cls', Float32MultiArray, queue_size=1) # tf_cls publish
        self.result_img = None
        self.msg = Float32MultiArray()

    def run(self,img):
        self.results = self.model(img,verbose=False)
        self.result_img = self.results[0].plot()
        for r in self.results:
            self.msg.data = r.boxes.cls.cpu().numpy()
            self.cls_pub.publish(self.msg)
            # rospy.loginfo(self.msg)
        # print('finish')
    
    def TrafficLight_index(self,TrafficLight):
        pub = rospy.Publisher('TrafficLight', float, queue_size=10)
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            TrafficLight=self.results.boxes.cls


if __name__ == '__main__':
    rospy.init_node('yolo_test')
    cam0 = MoraiCamera(2)
    detect = TrafficLight()
    
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        detect.run(cam0.img)
        # result_img = np.concatenate([cam0.img, result[0].plot()])
        cv2.imshow('Orign-result',detect.result_img)
        cv2.waitKey(1)
        rate.sleep()