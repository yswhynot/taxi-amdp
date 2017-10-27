import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from taxi_amdp.srv import *
from taxi_amdp.planner import PositionAction
from taxi_amdp.planner import Map 
from taxi_amdp.planner import MDPNode
from taxi_amdp.planner import AMDP
import random

SIZE = 15
ACTION = 4

class Taxi:
    def __init__(self):
        rospy.init_node('taxi', anonymous=True)
        self.taxi_loc_pub = rospy.Publisher("/taxi_loc", Point, queue_size=1)
        self.pas_state_sub = rospy.Subscriber("/passenger", String, self.psg_state_cb)
        self.pas_state = "off"
        self.pas_loc = [0, 0]
        self.pas_des = [0, 0]

        self.amdp = AMDP()
        self.taxi_loc = self.random_loc()
        self.taxi_dir = random.randint(0, 3)
        self.current_state = "get"

    def random_loc(self):
        x = random.randint(0, SIZE - 1)
        y = random.randint(0, SIZE - 1)
        flag = False
        _map = self.amdp.mdp_map._map
        while not flag:
            if _map[x][y].north and _map[x][y].south and _map[x][y].west and _map[x][y].east:
                flag = True
            else:
                x = random.randint(0, SIZE - 1)
                y = random.randint(0, SIZE - 1)

        return [float(x), float(y)]

    def psg_state_cb(self, state):
        self.pas_state = state.data

    def wrap_direction(self, d):
        if d < 0:
            d += ACTION
        elif d >= ACTION:
            d -= ACTION
        return d

    def noisy_direction(self, d, x, y):
        w = 0.2/3
        rand = random.uniform(0, 1)
        r = d
        if rand < w:
            r = self.wrap_direction(d+1)
        elif rand < 2*w:
            r = self.wrap_direction(d+2)
        elif rand < 3*w:
            r = self.wrap_direction(d+3)
        return self.amdp.mdp_map.valid_action(x, y, r)

    def update_taxi(self):
        step = 0.1
        x = self.taxi_loc[0]
        y = self.taxi_loc[1]
        #  print "x: %f, %d, y: %f, %d" % (x, int(x+step), y, int(y+step))
        if abs(x - int(x+0.05)) < (step/2) and abs(y - int(y+0.05)) < (step/2):
            self.taxi_dir = self.noisy_direction(self.amdp.mdp_map.policy_map[int(x+0.05)][int(y+0.05)], int(x+0.05), int(y+0.05))
        if self.taxi_dir == 0:
            self.taxi_loc[0] -= step
        elif self.taxi_dir == 1:
            self.taxi_loc[0] += step
        elif self.taxi_dir == 2:
            self.taxi_loc[1] += step
        elif self.taxi_dir == 3:
            self.taxi_loc[1] -= step

    def update_state(self):
        # observe the passenger's state
        rospy.wait_for_service("/pas_serv")
        rospy.wait_for_service("/pas_loc")
        try:
            state_srv = rospy.ServiceProxy("/pas_serv", State)
            self.pas_state = state_srv("request").state
            pas_loc_srv = rospy.ServiceProxy("/pas_loc", Location)
            res = pas_loc_srv("current")
            self.pas_loc = (res.pt[0], res.pt[1])
        except rospy.ServiceException, e:
            pass

        # map environment state to abstract states
        print "taxi: (%.2f, %.2f), pas: (%.2f, %.2f), des: (%.2f, %.2f)" % (self.taxi_loc[0], self.taxi_loc[1], self.pas_loc[0], self.pas_loc[1], self.pas_des[0], self.pas_des[1])
        print "taxi: %s, pas: %s" % (self.current_state, self.pas_state)
        if abs(self.taxi_loc[0] - self.pas_loc[0]) < 0.01 and abs(self.taxi_loc[1] - self.pas_loc[1]) < 0.01:
            if self.current_state == "get":
                self.current_state = "pickup"
            elif self.current_state == "pickup":
                if self.pas_state == "on":
                    self.current_state = "put"
            elif self.current_state == "put":
                res = pas_loc_srv("destination")
                self.pas_des = (res.pt[0], res.pt[1])
                self.amdp.mdp_map.reset_term(int(self.pas_des[0]+0.05), int(self.pas_des[1]+0.05))
                self.amdp.solve()
                return True
            return False
        elif abs(self.taxi_loc[0] - self.pas_des[0]) < 0.01 and abs(self.taxi_loc[1] - self.pas_des[1]) < 0.01:
            if self.current_state == "put":
                self.current_state = "putdown"
            elif self.current_state == "putdown":
                if self.pas_state == "off":
                    self.current_state = "get"
            return False
        else:
            if self.current_state == "get":
                self.amdp.mdp_map.reset_term(int(self.pas_loc[0]), int(self.pas_loc[1]))
                self.amdp.solve()
            return True

    
    def start(self):
        rospy.loginfo("Taxi start")
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.update_state():
                self.update_taxi()
            #  if self.current_state == "put":
                #  self.amdp.display()
            #  self.amdp.display()
            self.taxi_loc_pub.publish(Point(self.taxi_loc[1], self.taxi_loc[0], self.taxi_dir))
            rate.sleep()
