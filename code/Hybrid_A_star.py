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
        self.thetad= theta
        self.h = 0
        self.g = 10000
        self.parent = None
        self.children = []
        self.isobstacle = False
        self.isstart = False
        self.isgoal = False
        self.speed = 0.5
        self.deltatheta = 0.1

    def cost(self, current):
        if self.isobstacle == True or current.isobstacle == True:
            return 10000
        else:
            return math.sqrt((self.xd - current.xd) ** 2 + (self.yd - current.yd) ** 2 + (self.thetad - current.thetad))

    def euclidean(self,goal):
        return math.sqrt((self.xd - goal.x) ** 2 + (self.yd - goal.y) ** 2)

    def successor(self,goal):
        steering = ['left', 'right']
        gear = ['drive','reverse']

        for direction in steering:
            for g in gear:
                if direction == 'left':
                    d = 1
                elif direction == 'right' :
                    d = -1
                else:
                    d = 0
                if g == 'd':
                    w = 1
                else:
                    w = -1
                a = State(self.x,self.y,self.theta)
                a.thetad = self.theta + d * self.deltatheta
                a.xd = self.x + w * self.speed * math.cos(self.thetad)
                a.yd = self.y + w * self.speed * math.sin(self.thetad)
                a.g = self.g + self.cost(a)
                a.heuristic(goal)
                a.roundstate()
                self.children.append(a)

    def roundstate(self):
        self.x = math.floor(self.xd)
        self.y = math.floor(self.yd)

    def heuristic(self,goal):
        self.h = self.euclidean(goal)

class Astar:
    def __init__(self):
        self.openlist = set()
        self.closelist = set()
        self.path =[]

    def run(self, start ,goal):
        self.openlist.add(start)
        start.h = start.heuristic(goal)
        while True:
            current = self.min_state()
            self.openlist.remove(start)
            self.closelist.add(start)

            if current.isgoal:
                break
            for child in current.children:
                if child in self.closelist:
                    continue
                if child not in self.openlist:
                    self.openlist.add(child)
                elif child.g > current.g + current.cost(child):
                    child.g = current.g +  current.cost(child)
                    child.parent = current
        self.get_backpointer_list(goal,start)

    def get_backpointer_list(self,current, start):
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
            return min(self.openlist, key=lambda x: x.g+x.h)

max_x = 7
max_y = 6
state = [[State(j, i,0) for i in range(max_x)] for j in range(max_y)]
plt.title("A star")
state[1][3].isstart = True # set start point
state[5][3].isgoal = True  # set goal point
S = state[2][1]
G = state[5][5]
obstaclelist = [ [3, 3], [4, 3]]
startTime = time.time()
astar = Astar()
astar.run(S,G)
x = []
y = []
for n in astar.path:
    x.append(n.xd)
    y.append(n.yd)
print("It took %s seconds to run" % (time.time() - startTime))

for n in obstaclelist:
    state[n[0]][n[1]].isobstacle =True

plt.xlim((0, max_x))
plt.ylim((0, max_y))

plt.fill_between( np.array([S.x,S.x+1]), S.y, S.y+1, facecolor='green')
plt.fill_between( np.array([G.x,G.x+1]), G.y, G.y+1, facecolor='red')
plt.fill_between( np.array([3,5]), 3, 4, facecolor='black')
plt.grid()
plt.plot(x,y)
plt.show()
