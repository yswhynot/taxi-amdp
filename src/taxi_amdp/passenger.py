#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from taxi_amdp.srv import *

class Passenger:
    def __init__(self):
        rospy.init_node('taxi_passenger', anonymous=True)
        self.pas_state_pub = rospy.Publisher("/passenger", String, queue_size=1)
        self.pas_serv = rospy.Service("/pas_serv", State, self.pas_serv_cb)
        self.loc_srv = rospy.Service("/pas_loc", Location, self.pas_loc_cb)
        self.pas_state = "off"

        rospy.loginfo("Passenger init")

    def pas_loc_cb(self, req):
        if req.request is "current":
            return Point(0, 0, 0)
        elif req.request if "destination":
            return Point(15, 15, 0)

    def pas_serv_cb(self, state):
        if state is "request":
            return self.pas_state
        self.pas_state = state.state
        return "done"

    def start(self):
        rospy.loginfo("Passenger started")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.pas_state_pub.publish(String(self.pas_state))
            self.sleep()

