#! /usr/bin/env python

import cv2
import numpy as np
import rospy
from taxi_amdp.msg import StringList
from taxi_amdp.msg import PointList
from taxi_amdp.msg import Point2D
from geometry_msgs.msg import Point 
from std_msgs.msg import String

class TaxiMap:
    def __init__(self, size=15, res=30):
        rospy.init_node('taxi_canvas', anonymous=True)

        self.size = size
        self.resolution = res
        self.pas_count = 3
        self.pas_state = ["off", "off", "off"]
        self.pas_loc = [(100, 100), (100, 100), (100, 100)]
        self.taxi_loc = [100, 100]

        # draw the taxi map canvas
        self.img = np.full((size*res, size*res, 3), 255, np.uint8)

        self.taxi_loc_sub = rospy.Subscriber("/taxi_loc", Point, self.taxi_loc_cb)
        self.pas_loc_sub = rospy.Subscriber("/pas_loc", PointList, self.pas_loc_cb)
        self.pas_state_sub = rospy.Subscriber("/passenger", StringList, self.passenger_state_cb)
        #  self.pas_loc_sub = rospy.Subscriber("/
        rospy.loginfo("Taxi map init")

    def pas_loc_cb(self, p):
        for i in range(self.pas_count):
            if self.pas_state[i] == "off":
                self.pas_loc[i] = [p.points[i].x, p.points[i].y]
            elif self.pas_state[i] == "left":
                self.pas_loc[i] = [-1, -1]

    def passenger_state_cb(self, state):
        for i in range(self.pas_count):
            self.pas_state[i] = state.list[i]

    def update_passenger(self):
        for i in range(self.pas_count):
            if self.pas_state[i] == "on":
                self.pas_loc[i] = self.taxi_loc

    def taxi_loc_cb(self, loc):
        self.taxi_loc = [loc.x, loc.y]
        self.update_passenger()

    def empty_map(self):
        length = (self.size - 1) / 2
        self.img = np.full((self.size*self.resolution, self.size*self.resolution, 3), 255, np.uint8)
        cv2.rectangle(self.img, 
                (int((length+0.25) * self.resolution), 0), 
                (int((length+0.75) * self.resolution), int((length-0.5) * self.resolution)), 
                (0, 0, 0), -1)
        cv2.rectangle(self.img, 
                (int(1.25 * self.resolution), self.size*self.resolution - 1), 
                (int(1.75 * self.resolution), self.size*self.resolution - int((length-0.5)*self.resolution)), 
                (0, 0, 0), -1)
        cv2.rectangle(self.img, 
                (int((self.size - 2.75) * self.resolution), self.size*self.resolution - 1), 
                (int((self.size - 2.25) * self.resolution), self.size*self.resolution - int((length-0.5)*self.resolution)), 
                (0, 0, 0), -1)

    def show_passenger(self):
        r = int(0.5*self.resolution/2)
        for i in range(self.pas_count):
            cv2.circle(self.img, (int(self.pas_loc[i][0]*self.resolution), int(self.pas_loc[i][1]*self.resolution)), r, (20, 20, 200), -1)

    def show_taxi(self):
        r = int(0.6*self.resolution/2)
        cv2.rectangle(self.img, (int(self.taxi_loc[0]*self.resolution) - r, int(self.taxi_loc[1]*self.resolution) - r),
                (int(self.taxi_loc[0]*self.resolution) + r, int(self.taxi_loc[1]*self.resolution) + r), (200, 20, 20), -1)

    def show_map(self):
        self.empty_map()
        self.show_taxi()
        self.show_passenger()
        cv2.imshow("Taxi Map", self.img)
        cv2.waitKey(100)

    def start(self):
        rospy.loginfo("Taxi map started")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.show_map()
            rate.sleep()
