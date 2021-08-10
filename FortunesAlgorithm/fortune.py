import random
from PIL import Image, ImageDraw
import heapq
from dataclasses import dataclass, field
import math


class Voronoi():
    def __init__(self, points, bounding_box = None):
        self.lines = []
        self.arc_tree = None

        # Creates bounding box if none given
        if bounding_box == None:
            self.x0 = 0
            self.x1 = 500
            self.y0 = 0
            self.y1 = 500
            self.box_vert = [(self.x0, self.y0), (self.x0, self.y1), (self.x1, self.y1), (self.x1, self.y0)]
        else:
            self.x0 = min(bounding_box, key=lambda x: x[0])[0]
            self.x1 = max(bounding_box, key=lambda x: x[0])[0]
            self.y0 = min(bounding_box, key=lambda x: x[1])[1]
            self.y1 = max(bounding_box, key=lambda x: x[1])[1]
            self.box_vert = bounding_box
            # self.x0 = bounding_box[0]
            # self.x1 = bounding_box[1]
            # self.y0 = bounding_box[2]
            # self.y1 = bounding_box[3]
        
        self.box_lines = [self.box_vert[i]+self.box_vert[i+1] if i < len(self.box_vert)-1 else self.box_vert[i]+self.box_vert[0] for i in range(len(self.box_vert))]
        # adds points to "priority queue", points should have no duplicates
        self.points = []
        for p in points:
            new_point = Point(p[0], p[1])
            # if new_point.x > self.x0 and new_point.x < self.x1 and new_point.y > self.y0 and new_point.y < self.y1:
            if not self.isOutside(new_point.x, new_point.y):
                heapq.heappush(self.points, new_point)
        self.points_used = []
        for i in self.points:
            self.points_used.append(i)

            # if new_point.x < self.x0: self.x0 = new_point.x
            # if new_point.y < self.y0: self.y0 = new_point.y
            # if new_point.x > self.x1: self.x1 = new_point.x
            # if new_point.y > self.y1: self.y1 = new_point.y
        
        # dx = (self.x1 - self.x0 + 1) / 5.0
        # dy = (self.y1 - self.y0 + 1) / 5.0
        # self.x0 = self.x0 - dx
        # self.x1 = self.x1 + dx
        # self.y0 = self.y0 - dy
        # self.y1 = self.y1 + dy

        self.process()

    def process(self):
        while len(self.points) != 0:
            curr_event = heapq.heappop(self.points)
            if isinstance(curr_event, Event):
                if curr_event.valid:
                    seg = ArcSegment(curr_event.p)
                    self.lines.append(seg)

                    a = curr_event.a
                    if a.pprev is not None:
                        a.pprev.pnext = a.pnext
                        a.pprev.s1 = seg
                    if a.pnext is not None:
                        a.pnext.pprev = a.pprev
                        a.pnext.s0 = seg

                    if a.s0 is not None:
                        a.s0.Finish(curr_event.p)
                    if a.s1 is not None:
                        a.s1.Finish(curr_event.p)

                    if a.pprev is not None:
                        self.check_circle_event(a.pprev, curr_event.x)
                    if a.pnext is not None:
                        self.check_circle_event(a.pnext, curr_event.x)
            else:
                self.arc_insert(curr_event)
        self.finish_edges()

    def arc_insert(self, point):
        if self.arc_tree == None:
            self.arc_tree = Arc(point)
        else:
            i = self.arc_tree
            while i is not None:
                create, z = self.intersect(point,i)
                if create:
                    create, zz = self.intersect(point, i.pnext)
                    if i.pnext is not None and not create:
                        i.pnext.pprev = Arc(i.p, i, i.pnext)
                        i.pnext = i.pnext.pprev
                    else:
                        i.pnext = Arc(i.p, i)
                    i.pnext.s1 = i.s1

                    i.pnext.pprev = Arc(point, i, i.pnext)
                    i.pnext = i.pnext.pprev

                    i = i.pnext

                    segment = ArcSegment(z)
                    self.lines.append(segment)
                    i.pprev.s1 = i.s0 = segment

                    segment = ArcSegment(z)
                    self.lines.append(segment)
                    i.pnext.s0 = i.s1 = segment

                    self.check_circle_event(i, point.x)
                    self.check_circle_event(i.pprev, point.x)
                    self.check_circle_event(i.pnext, point.x)

                    return

                i = i.pnext
            
            i = self.arc_tree
            while i.pnext is not None:
                i = i.pnext
            i.pnext = Arc(point, i)

            x = self.x0
            y = (i.pnext.p.y + i.p.y) / 2
            start = Point(x, y)

            seg = ArcSegment(start)
            i.s1 = i.pnext.s0 = seg
            self.lines.append(seg)

    def intersect(self, point, i):
        if i == None:
            return False, None
        if i.p.x == point.x:
            return False, None

        a = 0
        b = 0

        if i.pprev is not None:
            a = (self.intersection(i.pprev.p, i.p, point.x)).y
        if i.pnext is not None:
            b = (self.intersection(i.p, i.pnext.p, point.x)).y

        if (i.pprev == None or a < point.y) and (i.pnext == None or point.y <= b):
            py = point.y
            px = (i.p.x**2 + (i.p.y-py)**2 - point.x**2)/(2*i.p.x - 2*point.x)
            return True, Point(px,py)
        return False, None

    def check_circle_event(self, i, x0):
        # look for a new circle event for arc i
        if (i.e is not None) and (i.e.x != self.x0):
            i.e.valid = False
        i.e = None

        if (i.pprev is None) or (i.pnext is None): return

        flag, x, o = self.circle(i.pprev.p, i.p, i.pnext.p)
        if flag and (x > self.x0):
            i.e = Event(x, o, i)
            heapq.heappush(self.points, i.e)
            # self.event.push(i.e)

    def circle(self, a, b, c):
        # check if bc is a "right turn" from ab
        if ((b.x - a.x)*(c.y - a.y) - (c.x - a.x)*(b.y - a.y)) > 0: return False, None, None

        A = b.x - a.x
        B = b.y - a.y
        C = c.x - a.x
        D = c.y - a.y
        E = A*(a.x + b.x) + B*(a.y + b.y)
        F = C*(a.x + c.x) + D*(a.y + c.y)
        G = 2*(A*(c.y - b.y) - B*(c.x - b.x))

        if (G == 0): return False, None, None # Points are co-linear

        # point o is the center of the circle
        ox = 1.0 * (D*E - B*F) / G
        oy = 1.0 * (A*F - C*E) / G

        # o.x plus radius equals max x coord
        x = ox + math.sqrt((a.x-ox)**2 + (a.y-oy)**2)
        o = Point(ox, oy)
        
        return True, x, o

    def intersection(self, p0, p1, l):
        # get the intersection of two parabolas
        p = p0
        if (p0.x == p1.x):
            py = (p0.y + p1.y) / 2.0
        elif (p1.x == l):
            py = p1.y
        elif (p0.x == l):
            py = p0.y
            p = p1
        else:
            # use quadratic formula
            z0 = 2.0 * (p0.x - l)
            z1 = 2.0 * (p1.x - l)

            a = 1.0/z0 - 1.0/z1;
            b = -2.0 * (p0.y/z0 - p1.y/z1)
            c = 1.0 * (p0.y**2 + p0.x**2 - l**2) / z0 - 1.0 * (p1.y**2 + p1.x**2 - l**2) / z1

            py = 1.0 * (-b-math.sqrt(b*b - 4*a*c)) / (2*a)
            
        px = 1.0 * (p.x**2 + (p.y-py)**2 - l**2) / (2*p.x-2*l)
        res = Point(px, py)
        return res

    def finish_edges(self):
        l = self.x1 + (self.x1 - self.x0) + (self.y1 - self.y0)
        i = self.arc_tree
        while i.pnext is not None:
            if i.s1 is not None:
                p = self.intersection(i.p, i.pnext.p, l*2)
                i.s1.Finish(p)
            i = i.pnext
    
    def print_lines(self):
        it = 0
        for i in self.lines:
            it = it+1
            p0 = i.start
            p1 = i.end
            print(p0.x, p0.y, p1.x, p1.y)
    
    def output(self):
        res = []
        for i in self.lines:
            p0 = i.start
            p1 = i.end
            res.append((p0.x, p0.y, p1.x, p1.y))
        return res

    def display_output(self, draw, box=False):
        line = self.output()
        for i in line:
            draw.line(i, (0,0,0))
        if box:
            for i in self.box_lines:
                draw.line(i, (0,0,0))

    def bind(self, draw):
        out = self.output()
        ind = []
        for i in range(len(out))[::-1]:
            # print(i)
            if self.isOutside(out[i][0], out[i][1]) and self.isOutside(out[i][2], out[i][3]):
                # draw.line(out[i], (255,0,0))
                self.lines.remove(self.lines[i])
                # ind.append(i)
            elif self.isOutside(out[i][0], out[i][1]) or self.isOutside(out[i][2], out[i][3]):
                for j in self.box_lines:
                    x1 = out[i][0]
                    x2 = out[i][2]
                    y1 = out[i][1]
                    y2 = out[i][3]
                    x3 = j[0]
                    y3 = j[1]
                    x4 = j[2]
                    y4 = j[3]
                    d = ((x1-x2)*(y3-y4))-((y1-y2)*(x3-x4))
                    if d==0:
                        t = -1
                        u = -1
                    else:
                        t = (((x1-x3)*(y3-y4))-((y1-y3)*(x3-x4)))/(d)
                        u = (((x2-x1)*(y1-y3))-((y2-y1)*(x1-x3)))/(d)
                    if t <= 1 and t >= 0 and u <= 1 and u >= 0:
                        px1 = x1+t*(x2-x1)
                        py1 = y1+t*(y2-y1)
                        self.lines[i].start = Point(px1, py1) if self.isOutside(self.lines[i].start.x, self.lines[i].start.y) else self.lines[i].start
                        self.lines[i].end = Point(px1, py1) if self.isOutside(self.lines[i].end.x, self.lines[i].end.y) else self.lines[i].end
        # removed = False
        # for i in self.points_used[::-1]:
        #     if self.isOutside(i.x, i.y):
        #         self.points_used.remove(i)
        #         removed = True
        # if removed:
        #     self.points = [x for x in self.points_used]
        #     self.process()

    def isOutside(self, x1, y1):
        count = 0
        y2 = y
        x2 = max(self.box_vert, key=lambda x: x[0])[0]+100
        for j in self.box_lines:
            x3 = j[0]
            y3 = j[1]
            x4 = j[2]
            y4 = j[3]
            d = ((x1-x2)*(y3-y4))-((y1-y2)*(x3-x4))
            if d == 0:
                t = -1
                u = -1
            else:
                t = (((x1-x3)*(y3-y4))-((y1-y3)*(x3-x4)))/(d)
                u = (((x2-x1)*(y1-y3))-((y2-y1)*(x1-x3)))/(d)
            if t <= 1 and t >= 0 and u <= 1 and u >= 0:
                count+=1
        return count%2==0



class Point():
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __lt__(self, obj):
        # self < obj
        return self.x < obj.x

    def __le__(self, obj):
        # self <= obj
        return self.x <= obj.x

    def __eq__(self, obj):
        # self == obj.
        return self.x == obj.x

    def __ne__(self, obj):
        # self != obj.
        return self.x != obj.x

    def __gt__(self, obj):
        # self > obj.
        return self.x > obj.x

    def __ge__(self, obj):
        # self >= obj.
        return self.x >= obj.x

class Event():
    def __init__(self, x, point, a):
        self.x = x
        self.p = point
        self.a = a
        self.valid = True

    def __lt__(self, obj):
        # self < obj.
        return self.x < obj.x

    def __le__(self, obj):
        # self <= obj.
        return self.x <= obj.x

    def __eq__(self, obj):
        # self == obj.
        return self.x == obj.x

    def __ne__(self, obj):
        # self != obj.
        return self.x != obj.x

    def __gt__(self, obj):
        # self > obj.
        return self.x > obj.x

    def __ge__(self, obj):
        # self >= obj.
        return self.x >= obj.x

class Arc():
    def __init__(self, point, prev=None, nex=None):
        self.p = point
        self.pprev = prev
        self.pnext = nex
        self.e = None
        self.s0 = None
        self.s1 = None

class ArcSegment():
    def __init__(self, point):
        self.start = point
        self.end = None
        self.done = False

    def Finish(self, point):
        if not self.done:
            self.end = point
            self.done = True

if __name__ == "__main__":
    # points = generatePoints(50, 100, 'uniform')
    # points = [(100,100), (200,200), (200,100), (100,200)]
    # points = [(100,100), (200,200), (220,100)]
    maxX = 1500
    maxY = 900
    points = []
    for i in range(30):
        x = random.randint(0,maxX)
        y = random.randint(0,maxY)
        while((x,y) in points):
            x = random.randint(0,maxX)
            y = random.randint(0,maxY)
        points.append((x,y))
    bound = (0,maxX,0,maxY)
    bound = (150,maxX-150,150,maxY-150)
    box_vert = [(150, 150), (150, maxY-150), (300, maxY-500) ,(maxX-100, maxY-150), (maxX-100, 200), (maxX-200, 300)]
    # bound = [box_vert[i]+box_vert[i+1] if i < len(box_vert)-1 else box_vert[i]+box_vert[0] for i in range(len(box_vert))]
    test = Voronoi(points, bounding_box=box_vert)

    landscape = Image.new("RGB", (maxX,maxY), (255, 255, 255))
    draw = ImageDraw.Draw(landscape)
    
    
    test.bind(draw)
    # test.isOutside(200,200)
    test.display_output(draw, True)
    # draw.line([200,200,1400,200], (255,0,0))
    # test.print_lines()  
    for i in test.points_used:
        # print(i)
        draw.rectangle([i.x-1, i.y-1, i.x+1, i.y+1], (0,0,0))
    landscape.show()
    # test.print_lines()