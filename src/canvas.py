import cv2
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import String

class TaxiMap:
    def __init__(self, size=15, res=20):
        rospy.init_node('taxi_map', anonymous=True)
        
        self.size = size
        self.resolution = res
        self.pas_state = "off"
        self.pas_loc = (0, 0)

        # draw the taxi map canvas
        self.img = np.fill((size*res, size*res, 3), 255, np.uint8)
        
        self.taxi_loc_sub = rospy.Subscriber("/taxi_loc", Point, self.taxi_loc_cb)
        self.pas_serv = rospy.Service("/passenger", String, self.pas_serv_cb)
        rospy.loginfo("Taxi map init")

    def pas_serv_cb(self, state):
        self.pas_state = state.data


    def update_passenger(self):
        if self.pas_state is "on":
            self.pas_loc = self.taxi_loc

    def taxi_loc_sub(self, loc):
        self.taxi_loc = loc

    def empty_map(self):
        length = (self.size - 1) / 2
        self.img = np.fill((self.size*self.resolution, self.size*self.resolution, 3), 255)
        cv2.rectangle(self.img, 
                (0, length * self.resolution), 
                ((length+1) * self.resolution, length * self.resolution), 
                (0, 0, 0), -1)
        cv2.rectangle(self.img, 
                (self.size*self.resolution - 1, 1 * self.resolution), 
                (self.size*self.resolution - (length*self.resolution), 2 * self.resolution), 
                (0, 0, 0), -1)
        cv2.rectangle(self.img, 
                (self.size*self.resolution - 1, (self.size - 1) * self.resolution), 
                (self.size*self.resolution - (length*self.resolution), (self.size - 1) * self.resolution), 
                (0, 0, 0), -1)

    def show_map(self):
        cv2.imshow("Taxi Map", self.img)
        cv2.waitKey(10)

    def start(self):
        rospy.loginfo("Taxi map started")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.show_map()
            rate.sleep()
