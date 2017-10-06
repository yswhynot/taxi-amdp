#! /usr/bin/env python

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String

class TaxiMap:
	def __init__(self, size=15, res=20):
		rospy.init_node('taxi_canvas', anonymous=True)

		self.size = size
		self.resolution = res
		self.pas_state = "off"
		self.pas_loc = (100, 100)
		self.taxi_loc = (100, 100)

		# draw the taxi map canvas
		self.img = np.full((size*res, size*res, 3), 255, np.uint8)

		self.taxi_loc_sub = rospy.Subscriber("/taxi_loc", Point, self.taxi_loc_cb)
		self.pas_state_sub = rospy.Subscriber("/passenger", String, self.passenger_state_cb)
		rospy.loginfo("Taxi map init")

	def passenger_state_cb(self, state):
		self.pas_state = state.data

	def update_passenger(self):
		if self.pas_state is "on":
			self.pas_loc = self.taxi_loc

	def taxi_loc_cb(self, loc):
		self.taxi_loc = loc
		self.update_passenger()

	def empty_map(self):
		length = (self.size - 1) / 2
		self.img = np.full((self.size*self.resolution, self.size*self.resolution, 3), 255, np.uint8)
		cv2.rectangle(self.img, 
				(length * self.resolution, 0), 
				((length+1) * self.resolution, length * self.resolution), 
				(0, 0, 0), -1)
		cv2.rectangle(self.img, 
				(1 * self.resolution, self.size*self.resolution - 1), 
				(2 * self.resolution, self.size*self.resolution - (length*self.resolution)), 
				(0, 0, 0), -1)
		cv2.rectangle(self.img, 
				((self.size - 1) * self.resolution, self.size*self.resolution - 1), 
				((self.size - 2) * self.resolution, self.size*self.resolution - (length*self.resolution)), 
				(0, 0, 0), -1)

	def show_passenger(self):
		r = 5
		cv2.circle(self.img, (self.pas_loc[0], self.pas_loc[1]), r, (20, 20, 200), -1)

	def show_taxi(self):
		r = 10
		cv2.rectangle(self.img, (self.taxi_loc[0] - 10, self.taxi_loc[1] - 10),
				(self.taxi_loc[0] + 10, self.taxi_loc[1] + 10), (200, 20, 20), -1)

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
