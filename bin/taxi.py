#! /usr/bin/env python

import rospy
from taxi_amdp.taxi import Taxi

if __name__ == "__main__":
    try:
        taxi = Taxi()
        taxi.start()
    except rospy.ROSInterruptException:
        pass

