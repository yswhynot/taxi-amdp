import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from taxi_amdp.srv import *
import mdptoolbox
import numpy as _np
import random
import sys

action = ['north', 'south', 'east', 'west', 'get', 'put', 'root']

SIZE = 15
ACTION = 4

class PositionAction:
    north = True
    south = True
    west = True
    east = True

    is_term = False

class Map:

    def __init__(self):
        self._map = [[PositionAction() for i in range(SIZE)] for j in range(SIZE)]

        # build outer wall
        for i in range(SIZE):
            self._map[0][i].north = False
            self._map[i][0].west = False
            self._map[SIZE - 1][i].south = False
            self._map[i][SIZE - 1].east = False

        # build map with obstacles
        wall_length = (SIZE - 1) / 2
        for i in range(wall_length):
            self._map[i][(SIZE - 3) / 2].east = False
            self._map[i][(SIZE + 1) / 2].west = False
            self._map[i][(SIZE - 1) / 2].north = False

            self._map[SIZE - 1 - i][1].east = False
            self._map[SIZE - 1 - i][3].west = False
            self._map[SIZE - 1 - i][2].south = False

            self._map[SIZE - 1 - i][SIZE - 4].east = False
            self._map[SIZE - 1 - i][SIZE - 2].west = False
            self._map[SIZE - 1 - i][SIZE - 3].south = False

        # add terminal states
        # self._map[0][0].is_term = True
        # self._map(SIZE - 1, 0).is_term = True
        # self._map(0, SIZE - 1).is_term = True
        # self._map(SIZE - 1, SIZE - 1).is_term = True
    
    def reset_term(self, x, y):
        for i in range(SIZE):
            for j in range(SIZE):
                self._map[i][j].is_term = False
        self._map[x][y].is_term = True

    def set_term(self, term):
        if term == 0:
            self._map[0][0].is_term = True
        elif term == 1:
            self._map[0][SIZE - 1].is_term = True
        elif term == 2:
            self._map[SIZE - 1][0].is_term = True
        elif term == 3:
            self._map[SIZE - 1][SIZE - 1].is_term = True

    def set_state(self, state):
        if state is 'get':
            self._map[0][0].is_term = True
        elif state is 'put':
            self._map[SIZE-1][SIZE-1].is_term = True

    def get_neighbor_index(self, i):
        x = int(i / SIZE)
        y = i - x*SIZE

        # in sequence: 'north', 'south', 'east', 'west'
        result = [(x-1)*SIZE + y, (x+1)*SIZE + y, x*SIZE + y+1, x*SIZE + y-1]

        # wrap boulder
        if result[2] == (SIZE*SIZE):
            result[2] -= 1

        # change validation
        if not self._map[x][y].north:
            result[0] = i
        if not self._map[x][y].south:
            result[1] = i
        if not self._map[x][y].east:
            result[2] = i
        if not self._map[x][y].west:
            result[3] = i

        return result

    def get_opposite_action(self, a):
        if a == 0:
            return 1
        if a == 1:
            return 0
        if a == 2:
            return 3
        if a == 3:
            return 2

    def generate_mdp(self):
        p_wrong_dir = 0.1
        p_dir = 1 - 3*p_wrong_dir
        total_state = SIZE*SIZE

        # Transition matrix
        T = _np.zeros((ACTION, total_state, total_state))
        for i in range(total_state):
            neighbor = self.get_neighbor_index(i)

            for j in range(ACTION):
                for k in range(ACTION):
                    if j == k:
                        T[j, i, neighbor[k]] += p_dir
                    else:
                        T[j, i, neighbor[k]] += p_wrong_dir
        # print T[0, :, :]

        # Reward matrix
        R = _np.zeros((total_state, ACTION))
        # find the termination state
        term_state = -1
        for x in range(SIZE):
            for y in range(SIZE):
                if self._map[x][y].is_term:
                    term_state = x*SIZE + y
                    break
            if term_state != -1:
                break
        # update neighbor to termination state reward
        result = self.get_neighbor_index(term_state)
        for i in range(ACTION):
            if result[i] != term_state:
                R[result[i], self.get_opposite_action(i)] = 1

        return (T, R)

    def solve(self):
        T, R = self.generate_mdp()
        vi = mdptoolbox.mdp.ValueIteration(T, R, 0.95)
        vi.run()

        # parse policy
        policy = vi.policy
        self.policy_map = [[policy[x*SIZE + y] for y in range(SIZE)] for x in range(SIZE)]

    def display(self):
        policy_map = self.policy_map

        sys.stdout.write("\n")
        for x in range(SIZE):
            for y in range(SIZE):
                current_action = policy_map[x][y]
                if current_action == 0:
                    sys.stdout.write('^')
                elif current_action == 1:
                    sys.stdout.write('v')
                elif current_action == 2:
                    sys.stdout.write('>')
                elif current_action == 3:
                    sys.stdout.write('<')
                sys.stdout.write(' ')
            sys.stdout.write('\n')
        # self.print_map(2)

    def print_map(self, action):
        pmap = self._map

        for x in range(SIZE):
            for y in range(SIZE):
                if action == 0:
                    if pmap[x][y].north:
                        sys.stdout.write('o')
                    else:
                        sys.stdout.write('1')
                if action == 1:
                    if pmap[x][y].south:
                        sys.stdout.write('o')
                    else:
                        sys.stdout.write('1')
                if action == 2:
                    if pmap[x][y].east:
                        sys.stdout.write('o')
                    else:
                        sys.stdout.write('1')
                if action == 3:
                    if pmap[x][y].west:
                        sys.stdout.write('o')
                    else:
                        sys.stdout.write('1')
                sys.stdout.write(' ')
            sys.stdout.write('\n')

class MDPNode:
    def __init__(self, name, parent):
        self.name = name
        self.childs = []

        if parent is '':
            return
        parent.add_child(self)
        self.parents = [parent]

    def add_child(self, child):
        self.childs.append(child)

    def add_parent(self, parent):
        self.parents.append(parent)

    def solve(self):
        for child in self.childs:
            child.solve()

    def display(self):
        sys.stdout.write("\n")
        sys.stdout.write(self.name)
        for child in self.childs:
            child.display()

class AMDP:
    def __init__(self):
        # build mdp graph
        self.root = MDPNode('root', '')
        get_node = MDPNode('get', self.root)
        put_node = MDPNode('put', self.root)
        pick_node = MDPNode('pick', get_node)
        drop_node = MDPNode('drop', put_node)
        self.nav_node = MDPNode('nav', put_node)
        self.nav_node.add_parent(get_node)
        
        self.mdp_map = Map()
        self.nav_node.add_child(self.mdp_map)

    def solve(self):
        #  self.mdp_map.set_state(action)
        self.root.solve()

    def display(self):
        self.root.display()

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

    def update_taxi(self):
        step = 0.05
        x = self.taxi_loc[0]
        y = self.taxi_loc[1]
        if abs(x - int(x)) < step and abs(y - int(y)) < step:
            self.taxi_dir = self.amdp.mdp_map.policy_map[int(x)][int(y)]
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
            self.pas_state = state_srv("request")
            pas_loc_srv = rospy.ServiceProxy("/pas_loc", Location)
            res = pas_loc_srv("current")
            self.pas_loc = (res[0], res[1])
            res = pas_loc_srv("destination")
            self.pas_des = (res[0], res[1])
        except rospy.ServiceException, e:
            pass

        # map environment state to abstract states
        if self.taxi_loc is self.pas_loc:
            if self.current_state is "get":
                self.current_state = "pickup"
            elif self.current_state is "pickup":
                if self.pas_state is "on":
                    self.current_state = "put"
            elif self.current_state is "put":
                self.amdp.mdp_map.reset_term(int(self.pas_loc[0]), int(self.pas_loc[1]))
                self.amdp.solve()
                return True
            return False
        elif self.taxi_loc is self.pas_des:
            if self.current_state is "put":
                self.current_state = "putdown"
            elif self.current_state is "putdown":
                if self.pas_state is "off":
                    self.current_state = "get"
            return False
        else:
            self.amdp.mdp_map.reset_term(int(self.pas_loc[0]), int(self.pas_loc[1]))
            self.amdp.solve()
            return True

    
    def start(self):
        rospy.loginfo("Taxi start")
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.update_state():
                self.update_taxi()
            self.amdp.display()
            self.taxi_loc_pub.publish(Point(self.taxi_loc[1], self.taxi_loc[0], self.taxi_dir))
            rate.sleep()

        
#  if __name__ == '__main__':
    #  amdp = AMDP()
    #  amdp.solve('get')
    #  amdp.display()
