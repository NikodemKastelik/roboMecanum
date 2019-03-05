import numpy as np
import matplotlib.pyplot as plt
import time
import a_star
a_star.show_animation = False

class Robot():
    def __init__(self, px, py, radius, sensorrange):
        self.px = px
        self.py = py
        self.radius = radius
        self.max_speed = 50
        self.sensorrange = sensorrange

    def sense(self, obx, oby):
        sensedx = []
        sensedy = []

        for px, py in zip(obx, oby):
            if np.sqrt((self.px - px) ** 2 + (self.py - py) ** 2) < self.sensorrange:
                sensedx.append(px)
                sensedy.append(py)
        return sensedx, sensedy

    def motion(self, vx, vy, dt):
        self.px += vx * dt
        self.py += vy * dt

    def headTowards(self, px, py):
        dist_x = px - self.px
        dist_y = py - self.py

        vect_angle = np.arctan2(dist_y, dist_x)

        vx = np.cos(vect_angle) * self.max_speed
        vy = np.sin(vect_angle) * self.max_speed

        return vx, vy


def create_boundaries(px1, py1, px2, py2, safedist, reso):
    obx = []
    oby = []

    lower_x = min(px1, px2)
    upper_x = max(px1, px2)
    lower_y = min(py1, py2)
    upper_y = max(py1, py2)

    # create lower X bound
    for i in np.arange(lower_x - safedist, upper_x + safedist, reso):
        obx.append(float(i))
        oby.append(float(lower_y - safedist))

    # create upper X bound
    for i in np.arange(lower_x - safedist, upper_x + safedist, reso):
        obx.append(float(i))
        oby.append(float(upper_y + safedist))

    # create lower Y bound
    for i in np.arange(lower_y - safedist, upper_y + safedist, reso):
        obx.append(float(lower_x - safedist))
        oby.append(float(i))

    # create upper Y bound
    for i in np.arange(lower_y - safedist, upper_y + safedist, reso):
        obx.append(float(upper_x + safedist))
        oby.append(float(i))

    return obx, oby

def main():
    # units are cm
    sx = 100
    sy = 100

    ggx = 500
    ggy = 100
    astar_grid_reso = 10

    robot = Robot(sx, sy, radius = 15, sensorrange = 150)

    obst_x = [300, 300, 300, 300, 300, 320, 340, 360, 380, 380, 380, 380, 380]
    obst_y = [60,  80,  100, 120, 140, 150, 150, 160, 140, 120, 160, 140, 180]

    obx, oby, = create_boundaries(robot.px, robot.py, ggx, ggy, 100, robot.radius)
    objxy = set()

    show_animation = True
    for i in range(1000):

        sensedx, sensedy, = robot.sense(obst_x, obst_y)
        for x, y in zip(sensedx, sensedy):
            if not (x, y) in objxy:
                obx.append(x)
                oby.append(y)
                objxy.add((x,y))

        time_start = time.perf_counter()
        rx, ry = a_star.a_star_planning(float(robot.px), float(robot.py), ggx, ggy, obx, oby, astar_grid_reso, robot.radius)
        print("A star took: {} [s]", time.perf_counter() - time_start)

        if len(rx) > 2:
            idx = -3
        else:
            idx = 0
        next_x = rx[idx]
        next_y = ry[idx]

        vx, vy, = robot.headTowards(next_x, next_y)
        robot.motion(vx, vy, 0.25)

        if np.sqrt((robot.px - ggx) ** 2 + (robot.py - ggy) ** 2) < robot.radius:
            print("Done!")
            break

        if show_animation:
            plt.cla()
            ax = plt.gca()
            plt.plot(rx, ry, ".r", markersize = 3)
            plt.plot(obx, oby, ".g")
            plt.plot(ggx, ggy, "*r", markersize = 10)
            plt.plot(robot.px, robot.py, ".b")

            for x, y in zip(sensedx, sensedy):
                circle = plt.Circle((x, y), robot.radius / 2, fill = True)
                ax.add_artist(circle)

            circle = plt.Circle((robot.px, robot.py), robot.sensorrange, fill = False)

            ax.add_artist(circle)
            plt.pause(0.00001)

if __name__ == "__main__":
    main()
