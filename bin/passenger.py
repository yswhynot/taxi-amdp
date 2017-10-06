#! /usr/bin/env python

import rospy
from taxi_amdp.passenger import Passenger 

if __name__ == "__main__":
    try:
        p = Passenger()
        p.start()
    except rospy.ROSInterruptException:
        pass

