#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point 
from taxi_amdp.srv import *

class Passenger:
    def __init__(self):
        rospy.init_node('taxi_passenger', anonymous=True)
        self.pas_state_pub = rospy.Publisher("/passenger", String, queue_size=1)
        self.pas_loc_pub = rospy.Publisher("/pas_loc", Point, queue_size=1)
        self.pas_serv = rospy.Service("/pas_serv", State, self.pas_serv_cb)
        self.loc_srv = rospy.Service("/pas_loc", Location, self.pas_loc_cb)
        self.pas_state = "off"

        rospy.loginfo("Passenger init")

    def pas_loc_cb(self, req):
        if req.request == "current":
            return LocationResponse([1, 1])
        elif req.request == "destination":
            return LocationResponse([14, 14])

    def pas_serv_cb(self, state):
        if state.state == "request":
            return StateResponse(self.pas_state)
        self.pas_state = state.state
        rospy.loginfo("state changed to %s" % self.pas_state)
        return "done" 

    def start(self):
        rospy.loginfo("Passenger started")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.pas_loc_pub.publish(Point(1, 1, 0))
            self.pas_state_pub.publish(String(self.pas_state))
            rate.sleep()

