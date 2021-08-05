import random
from PIL import Image, ImageDraw
import heapq
from dataclasses import dataclass, field
import inspect

def generatePoints(x, y, gen_method, seed=None):
    random.seed(seed)
    if gen_method == 'uniform':
        start_pos = []
        xval = 10
        yval = 10
        while(xval < x):
            while(yval < y):
                start_pos.append((xval+20*random.random(), yval+20*random.random()))
                yval+=40
            yval=10
            xval+=40
        return(start_pos)

def fortunes(x, y, points):
    beachline = []
    event_queue = []
    for i in points:
        # temp = SiteEvent()
        heapq.heappush(event_queue, SiteEvent(i))

    for i in event_queue:
        if inspect.isclass(i, SiteEvent):
            print("Site Event")
        else:
            print("Intersect Event")

class SiteEvent():
    def __init__(self, xy):
        self.x = xy[0]
        self.y = xy[1]

    def __str__(self):
        return str(self.x)+" "+str(self.y)

    def __lt__(self, obj):
        return self.x < obj.x

    def __le__(self, obj):
        return self.x <= obj.x

    def __eq__(self, obj):
        return self.x == obj.x

    def __ne__(self, obj):
        return self.x != obj.x

    def __gt__(self, obj):
        return self.x > obj.x

    def __ge__(self, obj):
        return self.x >= obj.x

class IntersectEvent():
    def __init__(self, x):
        super().__init__()


if __name__ == "__main__":
    # points = generatePoints(50, 100, 'uniform')
    points = [(10,10), (20,20), (20,10), (10,20)]
    fortunes(50, 100, points)