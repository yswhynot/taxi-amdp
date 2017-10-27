import rospy
from geometry_msgs.msg import Point 
from taxi_amdp.msg import PointList
from taxi_amdp.msg import StringList
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
        self.pas_state_sub = rospy.Subscriber("/passenger", StringList, self.psg_state_cb)
        self.pas_count = 3
        self.pas_state = ["off", "off", "off"]
        self.pas_loc = [(1, 1), (1, 14), (14, 1)]  
        self.pas_des = [(14, 14), (14, 1), (1, 1)] 

        self.amdp = AMDP()
        self._map = Map()
        self.taxi_loc = self.random_loc()
        self.taxi_dir = random.randint(0, 3)
        self.current_state = "get"
        self.amdp.build_graph([int(self.taxi_loc[0]), int(self.taxi_loc[1])])
        self.amdp.solve()
        self.policy_map = self.amdp.get_next_policy()
        self.need_new_policy = True

    def random_loc(self):
        x = random.randint(0, SIZE - 1)
        y = random.randint(0, SIZE - 1)
        flag = False
        _map = self._map._map 
        while not flag:
            if _map[x][y].north and _map[x][y].south and _map[x][y].west and _map[x][y].east:
                flag = True
            else:
                x = random.randint(0, SIZE - 1)
                y = random.randint(0, SIZE - 1)

        return [float(x), float(y)]

    def psg_state_cb(self, state):
        for i in range(self.pas_count):
            self.pas_state[i] = state.list[i]

    def wrap_direction(self, d):
        if d < 0:
            d += ACTION
        elif d >= ACTION:
            d -= ACTION
        return d

    def noisy_direction(self, d, x, y):
        w = 0.02/3
        rand = random.uniform(0, 1)
        r = d
        if rand < w:
            r = self.wrap_direction(d+1)
        elif rand < 2*w:
            r = self.wrap_direction(d+2)
        elif rand < 3*w:
            r = self.wrap_direction(d+3)
        return self._map.valid_action(x, y, r)

    def update_taxi(self):
        step = 0.1
        x = self.taxi_loc[0]
        y = self.taxi_loc[1]
        #  print "x: %f, %d, y: %f, %d" % (x, int(x+step), y, int(y+step))
        if abs(x - int(x+0.05)) < (step/2) and abs(y - int(y+0.05)) < (step/2):
            self.taxi_dir = self.noisy_direction(self.policy_map[int(x+0.05)][int(y+0.05)], int(x+0.05), int(y+0.05))
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
        #  rospy.wait_for_service("/pas_serv")
        #  rospy.wait_for_service("/pas_loc")
        #  try:
            #  state_srv = rospy.ServiceProxy("/pas_serv", State)
            #  self.pas_state = state_srv("request").state
            #  pas_loc_srv = rospy.ServiceProxy("/pas_loc", Location)
            #  res = pas_loc_srv("current")
            #  self.pas_loc = (res.pt[0], res.pt[1])
        #  except rospy.ServiceException, e:
            #  pass

        # map environment state to abstract states
        #  print "taxi: (%.2f, %.2f), pas: (%.2f, %.2f), des: (%.2f, %.2f)" % (self.taxi_loc[0], self.taxi_loc[1], self.pas_loc[0], self.pas_loc[1], self.pas_des[0], self.pas_des[1])
        print "taxi: %s, pas: %s, %s, %s" % (self.current_state, self.pas_state[0], self.pas_state[1], self.pas_state[2])
        cid = self.amdp.get_current_pas()
        if(cid == -1):
            print "error get pas id"
            return False

        if abs(self.taxi_loc[0] - self.pas_loc[cid][0]) < 0.01 and \
                abs(self.taxi_loc[1] - self.pas_loc[cid][1]) < 0.01:
            if self.current_state == "get":
                self.current_state = "pickup"
            elif self.current_state == "pickup":
                if self.pas_state[cid] == "on":
                    self.current_state = "put"
                    self.need_new_policy = True
            elif self.current_state == "put":
                if self.need_new_policy:
                    self.policy_map = self.amdp.get_next_policy()
                    self.need_new_policy = False
                return True
            return False
        elif abs(self.taxi_loc[0] - self.pas_des[cid][0]) < 0.01 and abs(self.taxi_loc[1] - self.pas_des[cid][1]) < 0.01:
            if self.current_state == "put":
                self.current_state = "putdown"
            elif self.current_state == "putdown":
                if self.pas_state[cid] == "off":
                    self.current_state = "get"
            return False
        else:
            if self.current_state == "get":
                if self.need_new_policy:
                    self.policy_map = self.amdp.get_next_policy()
                    self.need_new_policy = False
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
