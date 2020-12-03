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
        self.length = 0
        self.path = np.zeros((3,3))
        self.t = 0
        self.p = 0
        self.q =0

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
        self.t = (-self.alpha + math.atan2(temp1,temp2))%(2*math.pi)
        self.p = math.sqrt(2 + self.d**2 - 2*math.cos(self.alpha -self.beta)
                           + 2*self.d*(math.sin(self.alpha) - math.sin(self.beta)))
        self.q = (self.beta - math.atan2(temp1, temp2))%(2*math.pi)
        self.length = (self.t+self.p+self.q)*self.radius


    def findpath(self,step):
        self.LSL()
        num = math.floor(self.length/step)
        self.path = np.zeros([num,3])
        self.path[0][0] = self.start.x
        self.path[0][1] = self.start.y
        self.path[0][2] = self.start.theta
        i=1
        ti = math.floor(self.t*self.radius/step)
        pi = math.floor((self.t+self.p)*self.radius/step)
        while i < num:
            if i<=ti:
                self.path[i] = self.action(self.path[i-1],'left',step)
            elif i<=pi:
                self.path[i] = self.action(self.path[i - 1], 'straight', step)
            else:
                self.path[i] = self.action(self.path[i - 1], 'left', step)
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
            end[0][1] = start[1] - length * math.sin(start[2])
            end[0][2] = start[2]
        return end

def main():
    start = state(0,0,math.pi/2)
    goal = state(-10,0,-math.pi/2)# radian
    d=dubins_curve(start,goal,2)
    d.findpath(0.01)
    plt.plot(start.x, start.y, 'kx')
    plt.arrow(start.x, start.y, start.x + math.cos(start.theta),start.y + math.sin(start.theta), width =0.05)
    plt.xlim(goal.x-10, start.x+10)
    plt.ylim( goal.y-10, start.y+10)
    plt.plot(goal.x, goal.y, 'kx')
    plt.annotate("", xy=(goal.x+ math.cos(goal.theta), goal.y+ math.sin(goal.theta)), xytext=(goal.x , goal.y ),
                 xycoords='data',
                 arrowprops=dict(facecolor='black', shrink=1)
                 )

    # plt.arrow(goal.x, goal.y, goal.x + math.cos(goal.theta), goal.y + math.sin(goal.theta), width=0.05)
    plt.plot(d.path[:,0],d.path[:,1],'b-')
    plt.grid(True)

    plt.title('Dubin\'s Curves Trajectory Generation')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()



if __name__ == '__main__':
    main()