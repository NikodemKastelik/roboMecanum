"""
A* grid based planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

"""

import matplotlib.pyplot as plt
import math
import time

show_animation = True


class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

def calc_node_idx(x, minx, reso):
    return math.floor((x - minx) / reso)

def calc_node_coord(ix, minx, reso):
    return minx + ix * reso# + reso / 2

def calc_final_path(ngoal, closedset, minx, miny, reso):
    # generate final course
    rx, ry = [calc_node_coord(ngoal.x, minx, reso)], [calc_node_coord(ngoal.y, miny, reso)]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(calc_node_coord(n.x, minx, reso))
        ry.append(calc_node_coord(n.y, miny, reso))
        pind = n.pind

    return rx, ry

def a_star_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    minnx = round(min(ox))
    minny = round(min(oy))

    nstart = Node(calc_node_idx(sx, minnx, reso), calc_node_idx(sy, minny, reso), 0.0, -1)
    ngoal = Node(calc_node_idx(gx, minnx, reso), calc_node_idx(gy, minny, reso), 0.0, -1)

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)

    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart

    while 1:
        c_id = min(
            openset, key=lambda o: openset[o].cost + calc_heuristic(ngoal, openset[o]))
        current = openset[c_id]

        # show graph
        if show_animation:  # pragma: no cover
            plt.plot(calc_node_coord(current.x, minnx, reso), calc_node_coord(current.y, minny, reso), "xc")
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)

        if current.x == ngoal.x and current.y == ngoal.y:
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i, _ in enumerate(motion):
            node = Node(current.x + motion[i][0],
                        current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)
            n_id = calc_index(node, xw, minx, miny)

            if n_id in closedset:
                continue

            if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue

            if n_id not in openset:
                openset[n_id] = node  # Discover a new node
            else:
                if openset[n_id].cost >= node.cost:
                    # This path is the best until now. record it!
                    openset[n_id] = node

    rx, ry = calc_final_path(ngoal, closedset, minnx, minny, reso)

    return rx, ry


def calc_heuristic(n1, n2):
    w = 1.0  # weight of heuristic
    d = w * math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
    return d


def verify_node(node, obmap, minx, miny, maxx, maxy):

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False

    try:
        dummy = obmap[(node.x)][(node.y)]
    except IndexError:
        print("Idx error!")
        print("Obmap:")
        [print("{}: {}".format(idx, row)) for idx, row in enumerate(obmap)]
        print("Node:")
        print(node)

    if obmap[(node.x)][(node.y)]:
        return False
    #except IndexError:
    #    print("Got index error for requested x:{} y:{}, obmap len: {}x{}".format(node.x, node.y, len(obmap), len(obmap[0])))

    return True

def calc_obstacle_map(ox, oy, reso, vr):
    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))

    xwidth = round((maxx - minx) / reso) + 1
    ywidth = round((maxy - miny) / reso) + 1

    obmap = [[False for i in range(ywidth)] for i in range(xwidth)]
    di = math.ceil(vr / reso)
    for iox, ioy in zip(ox, oy):
        ix = calc_node_idx(iox, minx, reso)
        iy = calc_node_idx(ioy, miny, reso)
        for x in range(ix - di, ix + di + 1):
            for y in range(iy - di, iy + di + 1):
                if 0 <= x < xwidth and 0 <= y < ywidth:
                    d = math.sqrt((iox - (iox + (x - ix) * reso)) ** 2 + (ioy - (ioy + (y - iy) * reso)) ** 2)
                    if d <= vr:
                        obmap[x][y] = True

    return obmap, 0, 0, xwidth, ywidth, xwidth, ywidth


def calc_index(node, xwidth, xmin, ymin):
    return node.x * xwidth + node.y


def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 23.2  # [m]
    sy = 17.5  # [m]
    gx = 55.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]
    robot_size = 2.5  # [m]

    ox, oy = [], []

    start = 0.0
    for i in range(-10, 70):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(70.0)
        oy.append(i)
    for i in range(71):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)


    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    rx, ry = a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    main()
