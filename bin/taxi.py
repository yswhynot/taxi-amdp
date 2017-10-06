#! /usr/bin/env python

import rospy
from taxi_amdp.canvas import TaxiMap

if __name__ == "__main__":
    try:
        taxi_map = TaxiMap()
        taxi_map.start()
    except rospy.ROSInterruptException:
        pass

