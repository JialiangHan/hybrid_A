from enum import Enum
import matplotlib.pyplot as plt
import random
import heapq
import bisect


class PointType(Enum):
    upper_end_point = 3
    intersection = 2
    lower_end_point = 1


class Point:
    def __init__(self, x, y, PointType=None, line1=None, line2=None):
        self.x = x
        self.y = y
        self.PointType = PointType
        self.line1 = line1
        self.line2 = line2

    def __str__(self):
        return "x: " + str(self.x) + ", y: " + str(self.y) + ", PointType: " + str(self.PointType)

    def __gt__(self, other):
        if self.y == other.y:
            if self.x == other.x:
                return self.PointType > other.PointType
            else:
                return self.x > other.x
        else:
            return self.y > other.y

    def __lt__(self, other):
        if self.y == other.y:
            if self.x == other.x:
                return self.PointType < other.PointType
            else:
                return self.x < other.x
        else:
            return self.y < other.y


class Segment:
    def __init__(self, t1, t2):
        # t1, t2 are class point
        if t1.y >= t2.y:
            self.upper_end_point = t1
            self.lower_end_point = t2
        else:
            self.upper_end_point = t2
            self.lower_end_point = t1
        self.upper_end_point.PointType = PointType.upper_end_point
        self.upper_end_point.line1 = self
        self.lower_end_point.PointType = PointType.lower_end_point
        self.lower_end_point.line1 = self
        self.k = 0
        self.b = 0
        self.slope()

    def __gt__(self, other):
        if self.lower_end_point.x == other.lower_end_point.x:
            return self.lower_end_point.y > other.lower_end_point.y
        return self.lower_end_point.x > other.lower_end_point.x

    def __lt__(self, other):
        if self.lower_end_point.x == other.lower_end_point.x:
            return self.lower_end_point.y < other.lower_end_point.y
        return self.lower_end_point.x < other.lower_end_point.x

    def slope(self):
        if self.upper_end_point.x == self.lower_end_point.x:
            self.k = float("inf")
            self.b = float("-inf")
        else:
            self.k = (self.lower_end_point.y - self.upper_end_point.y) / (
                    self.lower_end_point.x - self.upper_end_point.x)
            self.b = self.upper_end_point.y - self.k * self.upper_end_point.x

    def compute_intersection(self, line2):
        # x = (b2-b1)/(k1-k2),y=k1*x+b1
        line1 = self
        if line1.k - line2.k == 0:
            return Point(float('inf'), float('inf'))
        x = (line2.b - line1.b) / (line1.k - line2.k)
        y = line1.k * x + line1.b
        if line1.lower_end_point.y < y < line1.upper_end_point.y and line2.lower_end_point.y < y < line2.upper_end_point.y:
            return Point(x, y, PointType.intersection, line1=self, line2=line2)

    def __str__(self):
        return "l: " + str(self.lower_end_point) + ", u: " + str(self.upper_end_point)


class status:
    def __init__(self):
        self.status = []
        self.sweepline = 0

    def binaryappend(self, line):
        index = bisect.bisect_left(self.status, line)
        self.status.insert(index, line)

    def swap(self, index1, index2):
        self.status[index1], self.status[index2] = self.status[index2], self.status[index1]

    def add_check_intersection(self, segment, sweepline):
        self.sweepline = sweepline
        self.binaryappend(segment)
        index = self.status.index(segment)
        if 0 < index < len(self.status) - 1:
            inter1 = segment.compute_intersection(self.status[index - 1])
            inter2 = segment.compute_intersection(self.status[index + 1])
            if inter1.y >= self.sweepline and inter2.y >= self.sweepline:
                return inter1, inter2
            elif inter1.y >= self.sweepline > inter2.y:
                return inter1, None
            elif inter2.y >= self.sweepline > inter1.y:
                return inter2, None
            else:
                return (None, None)
        elif index == 0:
            if len(self.status) == 1:
                return None, None
            else:
                inter1 = segment.compute_intersection(self.status[index + 1])
                if inter1 is not None:
                    if inter1.y >= self.sweepline:
                        return inter1, None
                    else:
                        return None, None
        else:
            inter1 = segment.compute_intersection(self.status[index - 1])
            if inter1 is not None:
                if inter1.y >= self.sweepline:
                    return inter1, None
                else:
                    return None, None

    def delete_check_intersection(self, segment, sweepline):
        self.sweepline = sweepline
        index = self.status.index(segment)
        if 0 < index < len(self.status) - 1:
            inter1 = self.status[index - 1].compute_intersection(self.status[index + 1])
            if inter1.y >= self.sweepline:
                result = inter1
            else:
                result = None
        self.status.remove(segment)
        return result

    def swap_check_intersection(self, segment1, segment2, sweepline):
        self.sweepline = sweepline
        index1 = self.status.index(segment1)
        index2 = self.status.index(segment2)
        self.swap(index1, index2)
        if index1 == 0:
            try:
                inter1 = segment1.compute_intersection(self.status[index2 + 1])
                inter2 = None
            except IndexError:
                inter1 = None
                inter2 = None
        elif index2 == 0:
            try:
                inter1 = segment2.compute_intersection(self.status[index1 + 1])
                inter2 = None
            except IndexError:
                inter1 = None
                inter2 = None
        elif 0 < index1 < index2:
            try:
                inter1 = segment2.compute_intersection(self.status[index1 - 1])
            except IndexError:
                inter1 = None
            try:
                inter2 = segment1.compute_intersection(self.status[index2 + 1])
            except IndexError:
                inter2 = None
        else:
            try:
                inter1 = segment2.compute_intersection(self.status[index1 + 1])
            except IndexError:
                inter1 = None
            try:
                inter2 = segment1.compute_intersection(self.status[index2 - 1])
            except IndexError:
                inter2 = None
        if inter1 is not None and inter2 is not None:
            if inter1.y >= self.sweepline and inter2.y >= self.sweepline:
                return inter1, inter2
            elif inter1.y >= self.sweepline > inter2.y:
                return inter1, None
            elif inter2.y >= self.sweepline > inter1.y:
                return inter2, None
            else:
                return None, None
        elif inter1 is None and inter2 is not None:
            if inter2.y >= self.sweepline:
                return inter1, inter2
            else:
                return inter1, None
        elif inter1 is not None and inter2 is None:
            if inter1.y >= self.sweepline:
                return inter1, inter2
            else:
                return None, inter2
        else:
            return None, None


class event_queue:
    def __init__(self):
        self.h = []

    def init(self, segments):
        pointlist = []
        for segment in segments:
            pointlist.append(segment.upper_end_point)
            pointlist.append(segment.lower_end_point)
        for point in pointlist:
            self.h.append(point)
        heapq.heapify(self.h)

    def push(self, point):
        if point is None:
            return
        else:
            heapq.heappush(self.h, point)

    def pop(self):
        return heapq.heappop(self.h)

    def size(self):
        return len(self.h)


class PlaneSweep:
    def __init__(self, input):
        self.event_queue = event_queue()
        self.event_queue.init(input)
        self.status = status()
        self.intersection = []

    def run(self):
        while self.event_queue.size() > 0:
            current_event = self.event_queue.pop()
            if current_event.PointType == PointType.intersection:
                self.intersection.append(current_event)
            self.process_event(current_event)

        return self.intersection

    def process_event(self, event):
        sweepline = event.y
        if event.PointType == PointType.lower_end_point:
            inter1, inter2 = self.status.add_check_intersection(event.line1, sweepline)
            self.event_queue.push(inter1)
            self.event_queue.push(inter2)
        elif event.PointType == PointType.upper_end_point:
            inter1 = self.status.delete_check_intersection(event, sweepline)
            self.event_queue.push(inter1)
        else:
            inter1, inter2 = self.status.swap_check_intersection(event.line1, event.line2, sweepline)
            self.event_queue.push(inter1)
            self.event_queue.push(inter2)


def main():
    # specify max range for x and y
    xmin, xmax, ymin, ymax = 0, 10, 0, 10
    # specify number of points
    n = 10
    points = []
    segments = []
    # generate sites
    random.seed(4)
    for i in range(n):
        points.append(Point(random.randint(xmin + 1, xmax - 1), random.randint(ymin + 1, ymax - 1)))
    # generate segments
    for i in range(0, n, 2):
        segments.append(Segment(points[i], points[i + 1]))
    # draw segments
    for segment in segments:
        plt.plot([segment.upper_end_point.x, segment.lower_end_point.x],
                 [segment.upper_end_point.y, segment.lower_end_point.y], 'black')
    planesweep = PlaneSweep(segments)
    intersections = planesweep.run()
    # draw intersections
    for intersection in intersections:
        plt.plot(intersection.x, intersection.y, 'bo', ms=5)
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    plt.show()


if __name__ == '__main__':
    main()
