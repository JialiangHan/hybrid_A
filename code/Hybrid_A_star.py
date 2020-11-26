import math
import time
from matplotlib import pyplot as plt
import numpy as np


class State:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.xd = x
        self.yd = y
        self.thetad = 0#change of angle
        self.h = 0
        self.g = 0
        self.f = 0
        self.astar_g = 0
        self.astar_h = 0
        self.astar_f = 0
        self.parent = None
        self.children = []
        self.isobstacle = False
        self.isstart = False
        self.isgoal = False
        self.speed = 0.8
        self.deltatheta = 0.6
        self.get_f()

    def cost(self, current):
        if self.isobstacle == True or current.isobstacle == True:
            return 10000
        elif abs(math.atan2(self.yd - current.yd, self.xd - current.xd) - current.thetad) < math.pi/2:  # drive
            return 1 * self.arclength(current) + 1.5 * self.deltaangle( current)
        else:  # reverse
            return 2 * self.arclength(current) + 1.5 * self.deltaangle(current)

    def arclength(self, current):
        return math.sqrt((self.xd - current.xd) ** 2 + (self.yd - current.yd) ** 2)

    def deltaangle(self, current):
        return abs(self.thetad - current.thetad)

    def heuristic(self, goal):
        # def unconh(self, goal):
        #
        # def conh(self, goal):
        #
        # self.h = max(unconh(), conh())
        self.h = self.arclength(goal) + self.deltaangle(goal)

    def get_f(self):
        self.f = self.g + self.h

    def get_f_astar(self):
        self.astar_f = self.astar_g + self.astar_h

    def successor(self, goal):
        self.children.clear()
        steering = ['left', 'straight', 'right']
        gear = ['drive', 'reverse']
        max_x = 7
        max_y = 6
        for direction in steering:
            for g in gear:
                if direction == 'left':
                    d = 1
                    w2 = 1.05# penalty coefficient for turning
                elif direction == 'right':
                    d = -1
                    w2 = 1.05# penalty coefficient for turning
                else:
                    d = 0
                    w2 = 1
                if g == 'drive':
                    w = 1
                    w1 = 1
                else:
                    w = -1
                    w1 = 2# penalty coefficent for reverse
                a = State(self.xd, self.yd, self.thetad)
                a.thetad = d * self.deltatheta# change of angle, positive for left turn, negative for right turn
                if a.thetad * self.thetad < 0:
                    w3 = 2## penalty coefficent for change of direction
                else:
                    w3 = 1
                a.theta = self.theta + a.thetad
                tempx = self.xd + w * self.speed * math.cos(a.theta)
                tempy = self.yd + w * self.speed * math.sin(a.theta)
                if 0 <= tempx <= max_x and 0 <= tempy <= max_y:
                    a.xd = tempx
                    a.yd = tempy
                    a.g = self.g + w1 * w2 * w3 *self.cost(a)
                    a.heuristic(goal)
                    a.roundstate()
                    a.get_f()
                    if a.x == goal.x and a.y == goal.y:
                        a.isgoal = True
                    a.parent = self
                    self.children.append(a)

    def roundstate(self):
        self.x = math.floor(self.xd)
        self.y = math.floor(self.yd)
        #self.theta = math.floor(self.thetad)

    def ED(self, current):
        return math.sqrt((self.x - current.x) ** 2 + (self.y - current.y) ** 2)

    def astarsuccessor(self):
        for i in range(- 1, 2):
            for j in range(- 1, 2):
                if (self.x + i in range(0, max_y)) and (self.y + j in range(0, max_x)):
                    if self.x + i == self.x and self.y + j == self.y:
                        continue
                    if not state[self.x + i][self.y + j].isobstacle:
                        self.children.append(state[self.x + i][self.y + j])

class HAstar:
    def __init__(self):
        self.openlist = set()
        self.closelist = set()
        self.openlist_astar = set()
        self.closelist_astar = set()
        self.path = []

    def run(self, start, goal):
        self.openlist.add(start)
        start.h = self.unconstrained_h(start,goal)
        start.get_f()
        while True:
            current = self.min_state()
            if current == -1:
                break
            current.successor(goal)
            self.openlist.remove(current)
            self.closelist.add(current)

            if current.isgoal:
                break
            else:
                for child in current.children:
                    if child.x == current.x and child.y == current.y:
                        # if child and current are in same cell
                        g = current.g + child.cost(current)
                        tiebreaker = 2.5
                        if child.f > current.f + tiebreaker:
                            continue
                        if (not self.exist(child, self.openlist)) or g < child.g:
                            child.parent = current
                            child.g = g
                            child.h = self.unconstrained_h(child,goal)
                            child.get_f()
                            if self.exist(child, self.openlist):
                                for n in self.openlist:
                                    if n.x == child.x and n.y == child.y:
                                        self.openlist.remove(n)
                                        self.closelist.add(n)
                                        self.openlist.add(child)
                                        break
                            else:
                                self.openlist.add(child)
                    elif not self.exist(child, self.closelist):
                        g = current.g + child.cost(current)
                        if (not self.exist(child, self.openlist)) or g < child.g:
                            child.parent = current
                            child.g = g
                            child.h = self.unconstrained_h(child,goal)
                            child.get_f()
                            if not self.exist(child, self.openlist):
                                self.openlist.add(child)

        self.get_backpointer_list(goal, start)

    def unconstrained_h(self,current,goal):
        offset = math.sqrt((current.xd - current.x) ** 2 + (current.yd - current.y) ** 2)
        if current in self.closelist_astar:
            return current.astar_g + offset
        else:
            self.astarrun(goal,current)
            return current.astar_g + offset

    def get_backpointer_list(self, current, start):
        self.path = [current]
        while True:
            s = current.parent
            self.path.append(s)
            if s == start:
                break
            else:
                current = current.parent

    def min_state(self):
        if not self.openlist:
            return -1
        else:
            return min(self.openlist, key=lambda x: x.f)

    def min_state_astar(self):
        if not self.openlist_astar:
            return -1
        else:
            return min(self.openlist_astar, key=lambda x: x.astar_f)

    def exist(self, current, list):
        for n in list:
            if n.x == current.x and n.y == current.y and n.theta == current.theta:
                return True


    def astarexist(self,current, list):
        for n in list:
            if n.x == current.x and n.y == current.y:
                return True

    def astarrun(self, start, goal):
        self.openlist_astar.add(start)
        start.astar_h = start.ED( goal)
        start.get_f_astar()
        while True:
            current = self.min_state_astar()
            current.astarsuccessor()
            self.openlist_astar.remove(current)
            self.closelist_astar.add(current)

            if current.x == goal.x and current.y == goal.y:
                break
            else:
                for child in current.children:
                    if self.astarexist(child, self.closelist_astar):
                        continue
                    if not self.astarexist(child, self.openlist_astar):
                        self.openlist_astar.add(child)
                        child.parent = current
                        child.astar_g = current.astar_g + child.ED( current)
                        child.astar_h = child.ED( goal)
                        child.get_f_astar()
                    elif current.astar_g + child.ED( current) < child.astar_g:
                        child.parent = current
                        child.astar_g = current.astar_g + child.ED(current)
                        child.astar_h = child.ED( goal)
                        child.get_f_astar()


max_x = 7
max_y = 6
state = [[State(j, i, 0) for i in range(max_x)] for j in range(max_y)]
plt.title("HA star")
state[1][3].isstart = True  # set start point
state[5][3].isgoal = True  # set goal point
state[5][3].theta = math.pi/2
S = state[1][3]
G = state[5][3]
obstaclelist = [[3, 3], [4, 3]]
for n in obstaclelist:
    state[n[0]][n[1]].isobstacle = True

startTime = time.time()
hastar = HAstar()
hastar.run(S, G)

x = []
y = []
for n in hastar.path:
    x.append(n.xd)
    y.append(n.yd)
print("It took %s seconds to run" % (time.time() - startTime))

plt.xlim((0, max_x))
plt.ylim((0, max_y))

plt.fill_between(np.array([S.x, S.x + 1]), S.y, S.y + 1, facecolor='green')
plt.fill_between(np.array([G.x, G.x + 1]), G.y, G.y + 1, facecolor='red')
plt.fill_between(np.array([3, 5]), 3, 4, facecolor='black')
plt.grid()
plt.plot(x, y)
plt.show()
