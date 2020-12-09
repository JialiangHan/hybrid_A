import matplotlib.pyplot as plt
import math
import numpy as np


class state:
    def __init__(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

class dubins_curve:
    def __init__(self,x1,x2,radius):
        self.start = x1
        self.goal = x2
        self.radius = radius
        self.alpha = 0
        self.beta = 0
        self.d = 0
        self.transfer()
        self.length = []
        self.path = np.zeros((3,3))
        self.t = []
        self.p = []
        self.q = []

    def transfer(self):
        dy = self.goal.y - self.start.y
        dx = self.goal.x - self.start.x
        D = math.sqrt(dy**2 + dx**2)
        self.d = D/self.radius
        theta = math.atan2(dy,dx)%(2*math.pi)
        self.alpha = (self.start.theta - theta)%(2*math.pi)
        self.beta = (self.goal.theta - theta)%(2*math.pi)

    def LSL(self):
        temp1 = math.cos(self.beta) - math.cos(self.alpha)
        temp2 = self.d + math.sin(self.alpha) - math.sin(self.beta)
        temp3 = 2 + self.d**2 - 2*math.cos(self.alpha -self.beta) + 2*self.d*(math.sin(self.alpha) - math.sin(self.beta))
        if temp3 < 0:
            print('No LSL path')
            self.t.append(float("inf"))
            self.p.append(float("inf"))
            self.q.append(float("inf"))
        else:
            self.t.append((-self.alpha + math.atan2(temp1,temp2))%(2*math.pi))
            self.p.append(math.sqrt(temp3))
            self.q.append((self.beta - math.atan2(temp1, temp2))%(2*math.pi))
        self.length.append((self.t[-1]+self.p[-1]+self.q[-1])*self.radius)

    def RSR(self):
        temp1 = math.cos(self.alpha) - math.cos(self.beta)
        temp2 = self.d - math.sin(self.alpha) + math.sin(self.beta)
        temp3 = 2 + self.d**2 - 2*math.cos(self.alpha -self.beta) + 2*self.d*(math.sin(self.beta) - math.sin(self.alpha))
        if temp3 < 0:
            print('No RSR path')
            self.t.append(float("inf"))
            self.p.append(float("inf"))
            self.q.append(float("inf"))
        else:
            self.t.append((self.alpha - math.atan2(temp1,temp2))%(2*math.pi))
            self.p.append(math.sqrt(temp3))
            self.q.append((-self.beta + math.atan2(temp1, temp2))%(2*math.pi))
        self.length.append((self.t[-1] + self.p[-1]+ self.q[-1]) * self.radius)

    def RSL(self):
        temp1 = - math.cos(self.alpha) - math.cos(self.beta)
        temp2 = self.d + math.sin(self.alpha) + math.sin(self.beta)
        temp3 = -2 + self.d ** 2 + 2 * math.cos(self.alpha - self.beta) + 2 * self.d * (math.sin(self.beta) + math.sin(self.alpha))
        if temp3 < 0:
            print('No RSL path')
            self.t.append(float("inf"))
            self.p.append(float("inf"))
            self.q.append(float("inf"))
        else:
            self.p.append(math.sqrt(temp3))
            self.t.append((-self.alpha + math.atan2(temp1,temp2)-math.atan2(-2,self.p[-1]))%(2*math.pi))
            self.q.append(((-self.beta)%(2*math.pi) + math.atan2(temp1, temp2) - math.atan2(-2,self.p[-1]))%(2*math.pi))
        self.length.append((self.t[-1] + self.p[-1]+ self.q[-1]) * self.radius)

    def LSR(self):
        temp1 = math.cos(self.alpha) + math.cos(self.beta)
        temp2 = self.d - math.sin(self.alpha) - math.sin(self.beta)
        temp3 = -2 + self.d ** 2 + 2 * math.cos(self.alpha - self.beta) - 2 * self.d * (math.sin(self.beta) + math.sin(self.alpha))
        if temp3 < 0:
            print('No LSR path')
            self.t.append(float("inf"))
            self.p.append(float("inf"))
            self.q.append(float("inf"))
        else:
            self.p.append(math.sqrt(temp3))
            self.t.append((self.alpha - math.atan2(temp1,temp2) + math.atan2(2,self.p[-1]))%(2*math.pi))
            self.q.append((self.beta%(2*math.pi) - math.atan2(temp1, temp2) + math.atan2(2,self.p[-1]))%(2*math.pi))
        self.length.append((self.t[-1] + self.p[-1]+ self.q[-1]) * self.radius)

    def RLR(self):
        temp1 = math.cos(self.alpha) - math.cos(self.beta)
        temp2 = self.d - math.sin(self.alpha) + math.sin(self.beta)
        temp3 =1/8*(6 - self.d ** 2 + 2 * math.cos(self.alpha - self.beta)+ 2 * self.d * (math.sin(self.alpha) - math.sin(self.beta)))
        if temp3 < 0:
            print('No RLR path')
            self.t.append(float("inf"))
            self.p.append(float("inf"))
            self.q.append(float("inf"))
        else:
            self.p.append(math.acos(temp3))
            self.t.append((self.alpha - math.atan2(temp1, temp2) + self.p[-1]/2) % (2 * math.pi))
            self.q.append((self.alpha - self.beta -self.t[-1]+ self.p[-1]) % (2 * math.pi))
        self.length.append((self.t[-1] + self.p[-1]+ self.q[-1]) * self.radius)

    def LRL(self):
        temp1 = - math.cos(self.alpha) + math.cos(self.beta)
        temp2 = self.d + math.sin(self.alpha) - math.sin(self.beta)
        temp3 = 1/8*(6 - self.d ** 2 + 2 * math.cos(self.alpha - self.beta) + 2 * self.d * (math.sin(self.alpha) - math.sin(self.beta)))
        if temp3 < 0:
            print('No LRL path')
            self.t.append(float("inf"))
            self.p.append(float("inf"))
            self.q.append(float("inf"))
        else:
            self.p.append(math.acos(temp3))
            self.t.append((- self.alpha + math.atan2(temp1, temp2) + self.p[-1]/2) % (2 * math.pi))
            self.q.append((-self.alpha + self.beta + 2 * self.p[-1]) % (2 * math.pi))
        self.length.append((self.t[-1] + self.p[-1]+ self.q[-1]) * self.radius)

    def findpath(self,step):
        self.LSL()
        self.RSR()
        self.RSL()
        self.LSR()
        self.RLR()
        self.LRL()
        m = min(self.length)
        n = self.length.index(m)
        if n == 0:
            direction = ['left','straight','left']
        elif n==1:
            direction = ['right','straight','right']
        elif n==3:
            direction = ['right','straight','left']
        elif n==2:
            direction = ['left','straight','right']
        elif n==4:
            direction = ['right','left','right']
        else:
            direction = ['left','right','left']
        num = math.floor(m/step)
        self.path = np.zeros([num,3])
        self.path[0][0] = self.start.x
        self.path[0][1] = self.start.y
        self.path[0][2] = self.start.theta
        i=1
        ti = math.floor(self.t[n]*self.radius/step)
        pi = math.floor((self.t[n]+self.p[n])*self.radius/step)
        while i < num:
            if i<=ti:
                self.path[i] = self.action(self.path[i-1],direction[0],step)
            elif i<=pi:
                self.path[i] = self.action(self.path[i - 1], direction[1], step)
            else:
                self.path[i] = self.action(self.path[i - 1], direction[2], step)
            i = i+1

    def action(self,start,direction,length):
        end = np.zeros([1, 3])
        if direction == "left":
            end[0][0] = start[0] + self.radius*math.sin(start[2] + length/self.radius) - self.radius*math.sin(start[2])
            end[0][1] = start[1] - self.radius*math.cos(start[2] + length/self.radius) + self.radius*math.cos(start[2])
            end[0][2] = start[2] + length/self.radius
        elif direction =='right':
            end[0][0] = start[0] - self.radius*math.sin(start[2] - length/self.radius) + self.radius*math.sin(start[2])
            end[0][1] = start[1] + self.radius*math.cos(start[2] - length/self.radius) - self.radius*math.cos(start[2])
            end[0][2] = start[2] - length/self.radius
        else:
            end[0][0] = start[0] + length * math.cos(start[2])
            end[0][1] = start[1] + length * math.sin(start[2])
            end[0][2] = start[2]
        return end

def main():
    start = state(0,0,math.radians(45))
    goal = state(2,0,math.radians(45))# radian
    R = 1
    step = 0.01
    xl = min(start.x, goal.x) - abs(start.x - goal.x)/10-5
    xh = max(start.x, goal.x) + abs(start.x - goal.x)/10+5
    yl = min(start.y, goal.y) - abs(start.y - goal.y)/10-5
    yh = max(start.y, goal.y) + abs(start.y - goal.y)/10+5
    d=dubins_curve(start,goal,R)
    d.findpath(step)
    plt.plot(start.x, start.y, 'kx')
    plt.arrow(start.x, start.y, math.cos(start.theta),math.sin(start.theta), width =0.05)
    plt.xlim( xl, xh)
    plt.ylim( yl, yh)
    plt.plot(goal.x, goal.y, 'kx')
    # #plt.annotate("", xy=(goal.x+ math.cos(goal.theta), goal.y+ math.sin(goal.theta)), xytext=(goal.x , goal.y ),
    #              xycoords='data',
    #              arrowprops=dict(facecolor='black', shrink=1)
    #              )
    plt.arrow(goal.x, goal.y, math.cos(goal.theta), math.sin(goal.theta), width=0.05)
    plt.plot(d.path[:,0],d.path[:,1],'b-')
    plt.grid(True)

    plt.title('Dubin\'s Curves Trajectory Generation')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()



if __name__ == '__main__':
    main()
