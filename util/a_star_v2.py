import numpy as np
import time
import threading

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from tkinter import *

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


class Main():
    def __init__(self):
        # units are cm
        startx = 100.0
        starty = 100.0

        self.goalx = 450.0
        self.goaly = 100.0

        self.robot = Robot(startx, starty, radius = 10, sensorrange = 100)

        self.obst_x = [300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 320.0, 340.0, 360.0, 380.0, 380.0, 380.0, 380.0, 380.0, 380.0, 380.0]
        self.obst_y = [  0.0,  20.0,  40.0,  60.0,  80.0, 100.0, 120.0, 140.0, 150.0, 150.0, 150.0, 160.0, 140.0, 120.0, 160.0, 140.0, 180.0, 200.0]

        self.rx = []
        self.ry = []

        self.obx, self.oby, = self.create_boundaries(self.robot.px, self.robot.py, self.goalx, self.goaly, 50, 3 * self.robot.radius)

        self.is_running = threading.Event()
        self.is_running.clear()

        width = 800
        height = 600
        self.root = Tk()
        self.root.geometry('{}x{}'.format(width, height))
        self.root.title('This is my root window')

        dpi = 100
        fig = plt.figure(figsize=(width / float(dpi), height / float(dpi)), dpi = dpi)
        plotcanvas = FigureCanvasTkAgg(fig, self.root)
        plotcanvas.get_tk_widget().grid(column=1, row=1)
        plotcanvas.mpl_connect('button_press_event', self.plot_clicked_handler)

        self.path_planning_thread = threading.Thread(target = self.path_planning_loop)
        self.ax = plt.gca()
        self.plot_path, = self.ax.plot([], [], ".r", markersize = 3)
        self.plot_bounds, = self.ax.plot([], [], ".g", markersize = 20)
        self.plot_goal, = self.ax.plot([], [], "*r", markersize = 10)
        self.plot_robot, = self.ax.plot([], [], "sb", markersize = 6)
        self.plot_sensor_circle = plt.Circle((self.robot.px, self.robot.py), self.robot.sensorrange, fill = False)
        self.ax.add_artist(self.plot_sensor_circle)
        self.plot_obst_circles = []

        self.ani = animation.FuncAnimation(fig, self.update_plot, interval = 10, blit = True)

        self.root.protocol('WM_DELETE_WINDOW', self.exit_handler)

    def plot_clicked_handler(self, event):
        x = float(event.xdata)
        y = float(event.ydata)
        self.obst_x.append(x)
        self.obst_y.append(y)

    def update_plot(self, i):
        self.plot_path.set_xdata(self.rx)
        self.plot_path.set_ydata(self.ry)

        self.plot_bounds.set_xdata(self.obx)
        self.plot_bounds.set_ydata(self.oby)

        self.plot_goal.set_xdata(self.goalx)
        self.plot_goal.set_ydata(self.goaly)

        self.plot_robot.set_xdata([self.robot.px])
        self.plot_robot.set_ydata([self.robot.py])

        self.ax.set_xlim(min(self.obx) - 100, max(self.obx) + 100)
        self.ax.set_ylim(min(self.oby) - 100, max(self.oby) + 100)

        self.plot_sensor_circle.center = (self.robot.px, self.robot.py)

        return [self.plot_path,
                self.plot_bounds,
                self.plot_goal,
                self.plot_robot,
                self.plot_sensor_circle,
                *self.plot_obst_circles]

    def exit_handler(self):
        self.ani.event_source.stop()
        plt.close("all")

        self.is_running.clear()
        self.path_planning_thread.join()

        self.root.destroy()

    def run(self):
        self.is_running.set()
        self.root.after(500, self.path_planning_thread.start)
        self.root.mainloop()

    def path_planning_loop(self):
        astar_grid_reso = 10
        objxy = set()

        while self.is_running.is_set():

            sensedx, sensedy, = self.robot.sense(self.obst_x, self.obst_y)
            for x, y in zip(sensedx, sensedy):
                if not (x, y) in objxy:
                    self.obx.append(x)
                    self.oby.append(y)
                    objxy.add((x,y))
                    circle = plt.Circle((x, y), self.robot.radius, fill = True)
                    self.ax.add_artist(circle)
                    self.plot_obst_circles.append(circle)

            time_start = time.perf_counter()
            try:
                self.rx, self.ry = a_star.a_star_planning(float(self.robot.px), float(self.robot.py),
                                                          self.goalx, self.goaly,
                                                          self.obx, self.oby,
                                                          astar_grid_reso, self.robot.radius)
            except Exception as e:
                errmsg = str(e)
                if errmsg == "min() arg is an empty sequence":
                    print("resizing")
                    self.obx, self.oby = self.resize_boundaries(self.obx, self.oby, 50)
                    for (x,y) in objxy:
                        self.obx.append(x)
                        self.oby.append(y)
                    continue
                else:
                    raise e
            print("A star took: {} [s]", time.perf_counter() - time_start)

            if len(self.rx) > 1:
                idx = -2
            else:
                idx = 0
            next_x = self.rx[idx]
            next_y = self.ry[idx]

            vx, vy, = self.robot.headTowards(next_x, next_y)
            self.robot.motion(vx, vy, 0.01)

            if np.sqrt((self.robot.px - self.goalx) ** 2 + (self.robot.py - self.goaly) ** 2) < self.robot.radius:
                print("Done!")
                break

    def resize_boundaries(self, obx, oby, safedist):
        reso = abs(obx[1] - obx[0])

        minx = min(obx)
        maxx = max(obx)
        miny = min(oby)
        maxy = max(oby)
        return self.create_boundaries(minx, miny, maxx, maxy, safedist, reso)

    def create_boundaries(self, px1, py1, px2, py2, safedist, reso):
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

if __name__ == "__main__":
    Main().run()
