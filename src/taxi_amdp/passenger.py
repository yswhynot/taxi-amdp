#! /usr/bin/env python

import rospy
from std_msgs.msg import String
#  from geometry_msgs.msg import Point 
from taxi_amdp.msg import Point2D
from taxi_amdp.msg import StringList
from taxi_amdp.msg import PointList
from taxi_amdp.srv import *

class Passenger:
    def __init__(self):
        rospy.init_node('taxi_passenger', anonymous=True)
        self.pas_state_pub = rospy.Publisher("/passenger", StringList, queue_size=1)
        self.pas_loc_pub = rospy.Publisher("/pas_loc", PointList, queue_size=1)
        self.pas_serv = rospy.Service("/pas_serv", State, self.pas_serv_cb)
        self.loc_srv = rospy.Service("/pas_loc", Location, self.pas_loc_cb)
        self.pas_state = ["off", "off", "off"]

        self.current = PointList([Point2D(1, 1), Point2D(1, 14), Point2D(14, 1)])
        self.destination = PointList([Point2D(14, 14), Point2D(14, 1), Point2D(1, 1)])

        rospy.loginfo("Passenger init")

    def pas_loc_cb(self, req):
        if req.request == "current":
            return LocationResponse(self.current)
        elif req.request == "destination":
            return LocationResponse(self.destination)

    def pas_serv_cb(self, state):
        if state.action == "request":
            res = StateResponse()
            res.state = self.pas_state
            return res 
        for i in range(3):
            self.pas_state[i] = state.state[i]
        print self.pas_state
        rospy.loginfo("state changed to %s, %s, %s" % (self.pas_state[0], self.pas_state[1], self.pas_state[2]))
        return "done" 

    def start(self):
        rospy.loginfo("Passenger started")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.pas_loc_pub.publish(self.current)
            self.pas_state_pub.publish(StringList(self.pas_state))
            rate.sleep()

