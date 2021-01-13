import matplotlib.pyplot as plt
import random


class node:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class line:
    def __init__(self, a, b):
        self.start = a
        self.end = b


class Voronoi:
    def __init__(self, sites, xmin, xmax, ymin, ymax):
        self.sites = sites
        self.bisetor = []
        self.edges = []
        self.vertices = []
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax

    def find_all_biscetor(self):
        for i in range(len(self.sites)):
            for j in range(i + 1, len(self.sites)):
                self.bisetor.append(self.get_bisector(self.sites[i], self.sites[j]))

    def get_bisector(self, a, b):
        # this k is for line ab
        # this kb is for bisector
        # this b is for bisector, function for bisector is y=kb*x+b
        if b.x == a.x:
            kb = 0
            b = 1 / 2 * (a.y + b.y)
            # find point for x=xmin
            yxmin = kb * self.xmin + b
            # find point for x=xmax
            yxmax = kb * self.xmax + b
            start = node(self.xmin, yxmin)
            end = node(self.xmin, yxmax)
            return line(start, end)
        elif a.y == b.y:
            # find point for y=ymin
            xymin = 1 / 2 * (a.x + b.x)
            # find point for y=ymax
            xymax = 1 / 2 * (a.x + b.x)
            start = node(xymin, self.ymin)
            end = node(xymax, self.ymax)
            return line(start, end)
        else:
            k = (b.y - a.y) / (b.x - a.x)
            b = (a.x + b.x) / (2 * k) + (a.y + b.y) / 2
            kb = -1 / k
            # find point for x=xmin
            yxmin = kb * self.xmin + b
            # find point for x=xmax
            yxmax = kb * self.xmax + b
            # find point for y=ymin
            xymin = (self.ymin - b) / kb
            # find point for y=ymax
            xymax = (self.ymax - b) / kb
            if yxmin in range(self.ymin, self.ymax):
                start = node(self.xmin, yxmin)
            else:
                start = node(self.xmax, yxmax)
            if xymin in range(self.xmin, self.xmax):
                end = node(xymin, self.ymin)
            else:
                end = node(xymax, self.ymax)
            return line(start, end)


def main():
    # specify max range for x and y
    xmin, xmax, ymin, ymax = 0, 10, 0, 10
    # specify number of sites
    n = 3
    sites = []
    # generate sites
    random.seed(1)
    for i in range(n):
        sites.append(node(random.randint(xmin + 1, xmax - 1), random.randint(ymin + 1, ymax - 1)))
    # draw sites
    for site in sites:
        plt.plot(site.x, site.y, 'bo', ms=5)
    V = Voronoi(sites, xmin, xmax, ymin, ymax)
    V.find_all_biscetor()
    # draw bisector
    for bisector in V.bisetor:
        plt.plot([bisector.start.x, bisector.end.x], [bisector.start.y, bisector.end.y])

    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)

    plt.show()


if __name__ == '__main__':
    main()
