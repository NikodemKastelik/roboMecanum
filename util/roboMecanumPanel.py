import tkinter as Tk
from tkinter import font

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.ticker as ticker
from matplotlib import style
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
style.use('ggplot')

import serial
import numpy as np
import threading
import queue
import time
import math
import multiprocessing

class AStarNode:
    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

class AStarAlgorithm:
    def __init__(self, reso, safedist):
        self.reso = reso
        self.safedist = safedist
        self._createVirtualBoundaries(-10.0, -10.0, 10.0, 10.0)

    def _createVirtualBoundaries(self, px1, py1, px2, py2):
        lower_x = min(px1, px2)
        upper_x = max(px1, px2)
        lower_y = min(py1, py2)
        upper_y = max(py1, py2)

        safedist = self.safedist
        reso = self.reso

        obx = []
        oby = []

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

        self._vobx = obx
        self._voby = oby

    def _resizeVirtualBoundaries(self):
        resize_val = 1.0
        minx = min(self._vobx) - resize_val
        maxx = max(self._vobx) + resize_val
        miny = min(self._voby) - resize_val
        maxy = max(self._voby) + resize_val
        self._createVirtualBoundaries(minx, miny, maxx, maxy)

    def _calculateNodeIndex(self, x, minx, reso):
        return math.floor((x - minx) / reso)

    def _calculateNodeCoordinates(self, ix, minx, reso):
        return minx + ix * reso

    def _calculateFinalPath(self, ngoal, closedset, minx, miny, reso):
        # generate final course
        rx = [self._calculateNodeCoordinates(ngoal.x, minx, reso)]
        ry = [self._calculateNodeCoordinates(ngoal.y, miny, reso)]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            rx.append(self._calculateNodeCoordinates(n.x, minx, reso))
            ry.append(self._calculateNodeCoordinates(n.y, miny, reso))
            pind = n.pind
        return rx, ry

    def _calculateHeuristic(self, n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
        return d

    def _verifyNode(self, node, obmap, minx, miny, maxx, maxy):
        if node.x < minx:
            return False
        elif node.y < miny:
            return False
        elif node.x >= maxx:
            return False
        elif node.y >= maxy:
            return False

        if obmap[(node.x)][(node.y)]:
            return False
        return True

    def _calculateObstacleMap(self, ox, oy, reso, vr):
        minx = round(min(ox))
        miny = round(min(oy))
        maxx = round(max(ox))
        maxy = round(max(oy))

        xwidth = round((maxx - minx) / reso) + 1
        ywidth = round((maxy - miny) / reso) + 1

        obmap = [[False for i in range(ywidth)] for i in range(xwidth)]
        di = math.ceil(vr / reso)
        for iox, ioy in zip(ox, oy):
            ix = self._calculateNodeIndex(iox, minx, reso)
            iy = self._calculateNodeIndex(ioy, miny, reso)
            for x in range(ix - di, ix + di + 1):
                for y in range(iy - di, iy + di + 1):
                    if 0 <= x < xwidth and 0 <= y < ywidth:
                        d = math.sqrt((iox - (iox + (x - ix) * reso)) ** 2 + (ioy - (ioy + (y - iy) * reso)) ** 2)
                        if d <= vr:
                            obmap[x][y] = True
        return obmap, 0, 0, xwidth, ywidth, xwidth, ywidth

    def _calcIndex(self, node, xwidth, xmin, ymin):
        return node.x * xwidth + node.y

    def _getMotionModel(self):
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

    def planPath(self, sx, sy, gx, gy, obx, oby):
        """
        gx: goal x position [m]
        gx: goal x position [m]
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        reso: grid resolution [m]
        rr: robot radius[m]
        """

        reso = self.reso
        rr = self.safedist
        ox = self._vobx + obx
        oy = self._voby + oby

        minnx = round(min(ox))
        minny = round(min(oy))

        nstart = AStarNode(self._calculateNodeIndex(sx, minnx, reso),
                           self._calculateNodeIndex(sy, minny, reso), 0.0, -1)

        ngoal = AStarNode(self._calculateNodeIndex(gx, minnx, reso),
                          self._calculateNodeIndex(gy, minny, reso), 0.0, -1)

        obmap, minx, miny, maxx, maxy, xw, yw = self._calculateObstacleMap(ox, oy, self.reso, self.safedist)

        motion = self._getMotionModel()

        openset, closedset = dict(), dict()
        openset[self._calcIndex(nstart, xw, minx, miny)] = nstart

        while 1:
            try:
                c_id = min(openset, key=lambda o: openset[o].cost + self._calculateHeuristic(ngoal, openset[o]))
            except Exception as e:
                if str(e) == "min() arg is an empty sequence": 
                    self._resizeVirtualBoundaries()
                raise e
            current = openset[c_id]

            if current.x == ngoal.x and current.y == ngoal.y:
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                break

            # Remove the item from the open set
            del openset[c_id]
            # Add it to the closed set
            closedset[c_id] = current

            # expand search grid based on motion model
            for i, _ in enumerate(motion):
                node = AStarNode(current.x + motion[i][0],
                                 current.y + motion[i][1],
                                 current.cost + motion[i][2],
                                 c_id)
                n_id = self._calcIndex(node, xw, minx, miny)

                if n_id in closedset:
                    continue

                if not self._verifyNode(node, obmap, minx, miny, maxx, maxy):
                    continue

                if n_id not in openset:
                    openset[n_id] = node  # Discover a new node
                else:
                    if openset[n_id].cost >= node.cost:
                        # This path is the best until now. record it!
                        openset[n_id] = node

        return self._calculateFinalPath(ngoal, closedset, minnx, minny, reso)

    def isGoalAchieved(self, px, py, gx, gy):
       return np.sqrt((px - gx) ** 2 + (py - gy) ** 2) <= self.safedist

class PathPlanner:
    def __init__(self, algorithm):
        self._algo = algorithm
        self._sx = 0
        self._sy = 0
        self._gx = 0
        self._gy = 0
        self._pathx = []
        self._pathy = []
        self._obx = []
        self._oby = []
        self._planning = False
        self._finished = False
        self._running = True
        self._algo_thread = threading.Thread(target = self._algorithmLoop)
        self._algo_thread.start()

    def _algorithmLoop(self,):
        while self._running:
            if self._planning:
                try:
                    time_start = time.perf_counter()
                    self._pathx, self._pathy = self._algo.planPath(self._sx, self._sy,
                                                                   self._gx, self._gy,
                                                                   self._obx, self._oby)
                    print("Astar took: {} [s]".format(time.perf_counter() - time_start))
                    self._planning = False
                    self._finished = True
                except ValueError:
                    continue
            else:
                time.sleep(0.001)

    def getStartPosition(self):
        return self._sx, self._sy

    def setStartPosition(self, sx, sy):
        self._sx = sx
        self._sy = sy

    def getGoalPosition(self):
        return self._gx, self._gy

    def setGoalPosition(self, gx, gy):
        self._gx = gx
        self._gy = gy

    def getPlannedPath(self):
        return self._pathx, self._pathy

    def isPlanningOngoingCheck(self):
        return self._planning

    def isPlanningFinished(self):
        return self._finished

    def clearPlanningFinished(self):
        self._finished = False

    def start(self):
        self._planning = True
        self._finished = False

    def stop(self):
        self._running = False
        self._algo_thread.join()

class SerialDevice:
    def __init__(self, port, baudrate = 115200, read_timeout = 0):
        self.dev = None
        self.port = port
        self.baudrate = baudrate
        self.read_timeout = read_timeout

    def open(self):
        self.dev = serial.Serial(self.port)
        self.dev.baudrate = self.baudrate
        self.dev.timeout = self.read_timeout

    def close(self):
        if self.dev:
            self.dev.close()
            self.dev = None

    def write(self, bytes):
        return self.dev.write(bytes)

    def read(self, byte_count):
        return self.dev.read(byte_count)

    def readLine(self):
        return self.dev.readline()

class SerialDeviceManager(SerialDevice):
    def __init__(self, port, baudrate = 115200, read_timeout = 0.001):
        super().__init__(port, baudrate, read_timeout)
        self._read_queue = queue.Queue()
        self._write_queue = queue.Queue()
        self._running = False
        self._bytes_send_per_access = 65535
        self._serial_access_delay = 0
        self._prefix = ""
        self._serial_device_loop_thread = threading.Thread(target = self._serialDeviceLoop)

    def _serialDeviceLoop(self):
        timeout_start = 0
        leftover = ""
        bytes_to_send = ""
        while self._running:
            if leftover and not bytes_to_send:
                bytes_to_send = leftover
                leftover = ""
            else:
                try:
                    bytes_to_send += self._write_queue.get(block=False)
                except queue.Empty:
                    pass

            if len(bytes_to_send) > self._bytes_send_per_access:
                leftover += bytes_to_send[self._bytes_send_per_access:]
                bytes_to_send = bytes_to_send[:self._bytes_send_per_access]

            if bytes_to_send and time.perf_counter() > timeout_start + self._serial_access_delay:
                self.write((self._prefix + bytes_to_send).encode())
                print("Sending: >>{}<<".format(bytes_to_send.replace("\n","\\n")))
                timeout_start = time.perf_counter()
                bytes_to_send = ""

            raw_line = self.readLine()
            try:
                line = raw_line.decode()
            except UnicodeDecodeError:
                print("Cannot decode bytes: {}".format(raw_line))
                line = ""
            if line:
                self._read_queue.put(line)

    def setBytesSendPerAccess(self, bytes_limit):
        self._bytes_send_per_access = bytes_limit

    def setPerAccessDelayMs(self, delay_ms):
        self._serial_access_delay = delay_ms

    def setPrefix(self, prefix):
        self._prefix = prefix

    def recv(self):
        try:
            line = self._read_queue.get(block=False)
        except queue.Empty:
            line = ""
        return line

    def send(self, data):
        self._write_queue.put(data)

    def start(self):
        self.open()
        self._running = True
        self._serial_device_loop_thread.start()

    def stop(self):
        self._running = False
        while self._serial_device_loop_thread.is_alive():
            pass
        self.close()

class Observable:
    def __init__(self):
        self._observers_callbacks = []

    def addObserverCallback(self, callback):
        self._observers_callbacks.append(callback)

    def fireCallbacks(self, *args):
        for callback in self._observers_callbacks:
            callback(*args)

class Model:

    ZIGBEE_MAX_BYTES_PER_ACCESS = 46
    ZIGBEE_DELAY_BETWEEN_ACCESS = 0.02
    ZIGBEE_PREFIX               = "P2P FB31 "

    def __init__(self, controller):
        self._controller = controller

        self._uart_mngr = SerialDeviceManager("/dev/ttyUSB0")
        self._uart_mngr.setBytesSendPerAccess(self.ZIGBEE_MAX_BYTES_PER_ACCESS)
        self._uart_mngr.setPerAccessDelayMs(self.ZIGBEE_DELAY_BETWEEN_ACCESS)
        self._uart_mngr.setPrefix(self.ZIGBEE_PREFIX)

        self._model_running = True
        self._uart_mngr_reader_loop_thread = threading.Thread(target = self._uartMngrReaderLoop)
        self._uart_mngr_reader_loop_thread.start()

        self._path_algo = AStarAlgorithm(reso = 0.1, safedist = controller.ROBOT_SIZE_RADIUS)
        self._path_planner = PathPlanner(self._path_algo)
        self.path_planner_observable = Observable()

        self._observables_loop_thread = threading.Thread(target = self._monitorObservablesLoop)
        self._observables_loop_thread.start()

    def _monitorObservablesLoop(self):
        while self._model_running:
            if self._path_planner.isPlanningFinished():
                self._path_planner.clearPlanningFinished()
                self.path_planner_observable.fireCallbacks(*self._path_planner.getPlannedPath())
            time.sleep(0.01)

    def _uartMngrReaderLoop(self):
        self._uart_mngr.start()
        while self._model_running:
            line = self._uart_mngr.recv()
            if line:
                self._controller.newUartDataHandler(line)
            time.sleep(0.001)

    def setGoalPosition(self, x, y):
        self._path_planner.setGoalPosition(x, y)

    def startPathPlanning(self):
        self._path_planner.start()

    def sendStringOverUart(self, data):
        if self._model_running:
            self._uart_mngr.send(data)

    def stop(self):
        self._model_running = False

        self._uart_mngr_reader_loop_thread.join()
        self._observables_loop_thread.join()

        self._uart_mngr.stop()
        self._path_planner.stop()

class MotorsFrameGraph:
    def __init__(self, parent_frame):
        self.DATA_SIZE = 200
        self.INTERVAL_MS = 1
        self._current_speed = [0, 0, 0 ,0]

        dpi = 70
        self.fig = plt.figure(figsize=(parent_frame['width'] / float(dpi), parent_frame['height'] / float(dpi)), dpi = dpi)
        self.fig.subplots_adjust(left   = 0.05,
                                 bottom = 0.05,
                                 right  = 0.95,
                                 top    = 0.95,
                                 wspace = 0.12,
                                 hspace = 0.1)

        ax_fl = self.fig.add_subplot(2, 2, 1)
        ax_fr = self.fig.add_subplot(2, 2, 2)
        ax_rl = self.fig.add_subplot(2, 2, 3)
        ax_rr = self.fig.add_subplot(2, 2, 4)
        self.ax = [ax_fr, ax_fl, ax_rr, ax_rl]

        for axis in self.ax:
            axis.set_ylim(-8000, 8000)
            axis.set_xlim(0, self.DATA_SIZE * (self.INTERVAL_MS / 1000))

        self.lines = []
        for axis in self.ax:
            line, = axis.plot([],[], '.r-')
            self.lines.append(line)

        self.plotdatas = []
        for axis in self.ax:
            plotdata = np.zeros([self.DATA_SIZE, 2])
            plotdata[:, 0] = np.linspace(-(self.DATA_SIZE) * self.INTERVAL_MS / 1000, 0, self.DATA_SIZE)
            self.plotdatas.append(plotdata)

        plotcanvas = FigureCanvasTkAgg(self.fig, parent_frame)
        plotcanvas.get_tk_widget().pack()
        self.anim = animation.FuncAnimation(self.fig, self._updatePlot, interval = self.INTERVAL_MS, blit = True)

    def _updatePlot(self, i):
        t = (i * self.INTERVAL_MS) / 1000

        for idx, speed in enumerate(self._current_speed):
            self.plotdatas[idx] = np.append(self.plotdatas[idx], [[t, speed]], axis = 0)
            self.plotdatas[idx] = np.delete(self.plotdatas[idx], (0), axis = 0)

        for axis, line, plotdata in zip(self.ax, self.lines, self.plotdatas):
            line.set_xdata(plotdata[:,0])
            line.set_ydata(plotdata[:,1])
            axis.set_xlim(plotdata[0, 0], plotdata[self.DATA_SIZE - 1, 0])

        return self.lines

    def setSpeed(self, motor_idx, speed):
        self._current_speed[motor_idx] = speed


class MotorControlPage(Tk.Frame):
    def __init__(self, *args, **kwargs):
        Tk.Frame.__init__(self, *args, **kwargs)

    def show(self):
        self._onShow()
        self.tkraise()
        self.lift()

    def _onShow(self):
        raise NotImplementedError

class PageVelocityVectorControl(MotorControlPage):
    def __init__(self, root, controller):
        MotorControlPage.__init__(self)

        self._controller = controller
        self._root = root
        self._is_button_pressed = False

        self._canvas = Tk.Canvas(self)
        self._canvas.pack(fill = Tk.BOTH, side = Tk.TOP, expand = True)

        root.update_idletasks()

        center_x = self._canvas.winfo_width() / 2
        center_y = self._canvas.winfo_height() / 2
        oval_radius_px = self._canvas.winfo_height() - 10
        self.oval = self._canvas.create_oval(center_x - oval_radius_px, center_y - oval_radius_px,
                                             center_x + oval_radius_px, center_y + oval_radius_px,
                                             fill = 'white')
        self._canvas.tag_bind(self.oval, '<Button-1>', self._clickedInCircleHandler)
        self._canvas.tag_bind(self.oval, '<ButtonRelease-1>', self._releasedCirceClickHandler)

        self.velocity_vector = self._canvas.create_line(center_x, center_y,
                                                        center_x, center_y,
                                                        arrow = Tk.LAST,
                                                        width = 4)

    def redraw(self):
        self._root.update_idletasks()
        center_x = self._canvas.winfo_width() / 2
        center_y = self._canvas.winfo_height() / 2
        oval_radius_px = (self._canvas.winfo_height() / 2) - 10
        self._canvas.coords(self.oval, (center_x - oval_radius_px, center_y - oval_radius_px,
                                        center_x + oval_radius_px, center_y + oval_radius_px))
        self._canvas.coords(self.velocity_vector, (center_x, center_y,
                                                   center_x, center_y))

    def _clickedInCircleHandler(self, event):
        if event is not None:
            mouse_x, mouse_y = event.x, event.y
        else:
            mouse_x = self._canvas.winfo_pointerx() - self._canvas.winfo_rootx()
            mouse_y = self._canvas.winfo_pointery() - self._canvas.winfo_rooty()

        center_x = self._canvas.winfo_width() / 2
        center_y = self._canvas.winfo_height() / 2

        p1x, p1y, p2x, p2y = self._canvas.coords(self.oval)
        radius = (p2x - p1x) / 2

        rel_x = mouse_x - center_x
        rel_y = mouse_y - center_y

        vector_len = np.sqrt(rel_x ** 2 + rel_y ** 2)
        if vector_len > radius:
            vector_len = radius
            vector_angle = np.arctan2(rel_y, rel_x)
            mouse_x = vector_len * np.cos(vector_angle) + center_x
            mouse_y = vector_len * np.sin(vector_angle) + center_y

        self._canvas.coords(self.velocity_vector, (center_x, center_y, mouse_x, mouse_y))

        vector_amount_0_to_100 = int((vector_len / radius) * 100)

        # rotate by 90 deg counter clockwise and then around new X axis
        vector_angle = -np.arctan2(rel_x, -rel_y)

        self._controller.setVelocityVector(vector_angle, vector_amount_0_to_100)

        self._buttonPressedHandler(event)

    def _buttonPressedHandler(self, event):
        delay_ms = 50
        if event is not None:
            self._is_button_pressed = True
            self._root.after(delay_ms, self._buttonPressedHandler, None)
        elif self._is_button_pressed:
            self._root.after(delay_ms, self._clickedInCircleHandler, None)

    def _releasedCirceClickHandler(self, event):
        self._is_button_pressed = False

    def _onShow(self):
        self.redraw()

class PagePidDirectControl(MotorControlPage):
    def __init__(self, controller):
        MotorControlPage.__init__(self)

        self._controller = controller
        self.small_font = Tk.font.Font(family = 'Consolas', size = 10)
        self.title_font = Tk.font.Font(family = 'Consolas', size = 18, weight = 'bold')

        column_labels = ["Kp", "Ki", "Kd", "Velocity"]
        row_labels = ["FR", "FL", "RR", "RL"]

        self.pid_entries = []
        self.velocity_entries = []
        for col_idx in range(len(column_labels)):
            pid_label = Tk.Label(self, text = column_labels[col_idx], font = self.title_font)
            pid_label.grid(row = 0, column = 2 * col_idx + 1, columnspan = 2, sticky = 'nsew', padx = 3, pady = 3)

        for row_idx in range(1, len(row_labels) + 1):
            motor_label = Tk.Label(self, text = row_labels[row_idx - 1], font = self.title_font)
            motor_label.grid(row = row_idx, column = 0, sticky = 'nsew');

            pid_entries = []
            for col_idx in range(len(column_labels)):
                entry = Tk.Entry(self, justify = Tk.CENTER, font = self.title_font)
                button_left = Tk.Button(self, text = "<", font = self.title_font, borderwidth = 3, relief = 'raised')
                button_right = Tk.Button(self, text = ">", font = self.title_font, borderwidth = 3, relief = 'raised')

                button_left.bind('<Button-1>', lambda event, idx = col_idx: self._buttonClickedHandler(event, "decrement {}".format(idx)))
                button_right.bind('<Button-1>', lambda event, idx = col_idx: self._buttonClickedHandler(event, "increment {}".format(idx)))

                button_left.bind('<ButtonRelease-1>', lambda event, idx = col_idx: self._buttonReleasedHandler(event, "decrement {}".format(idx)))
                button_right.bind('<ButtonRelease-1>', lambda event, idx = col_idx: self._buttonReleasedHandler(event, "increment {}".format(idx)))

                entry.grid(row = row_idx, column = 2 * col_idx + 1, columnspan = 2, sticky = 'nsew', padx = 20, pady = 3)
                #button_left.grid(row = 2, column = 2 * col_idx, sticky = 'nsew', padx = 20, pady = 3)
                #button_right.grid(row = 2, column = 2 * col_idx + 1, sticky = 'nsew', padx = 20, pady = 3)

                entry.insert(0, "0")

                if col_idx == len(column_labels) - 1:
                    self.velocity_entries.append(entry)
                else:
                    pid_entries.append(entry)
            self.pid_entries.append(pid_entries)

        button = Tk.Button(self, text = "GO!", font = self.title_font, borderwidth = 3, relief = 'raised')
        button.bind('<Button-1>', lambda event, idx = col_idx: self._buttonClickedHandler(event, "go"))
        button.grid(row = len(row_labels) + 2, column = int(len(column_labels) / 2) + 1, columnspan = 4, sticky = 'nsew', padx = 20, pady = 5)

        for i in range(len(row_labels) + 2):
            self.rowconfigure(index = i, weight = 1, minsize = 30)

        self.columnconfigure(index = 0, weight = 1, minsize = 50)
        for i in range(1, 2 * len(column_labels) + 1):
            self.columnconfigure(index = i, weight = 1)

    def _buttonClickedHandler(self, event, metadata):
        if metadata.startswith("increment"):
            entry_idx = int(metadata[-1])
            self._controller.incrementPidParamEntryHandler(entry_idx)
            self._buttonPressedHandler(event, metadata)

        elif metadata.startswith("decrement"):
            entry_idx = int(metadata[-1])
            self._controller.decrementPidParamEntryHandler(entry_idx)
            self._buttonPressedHandler(event, metadata)

        elif metadata == "go":
            self._controller.setPidPoint()

        else:
            print("Unsupported metadata in _buttonClickedHandler: {}".format(metadata))

    def _buttonPressedHandler(self, event, metadata):
        if event is not None:
            self._button_pressed_counter = 0
            self._is_button_pressed = True
            self.root.after(250, self._buttonPressedHandler, None, metadata)
        elif self._is_button_pressed:
            delay_ms = 100
            if self._button_pressed_counter > 10:
                delay_ms = 25
            self.root.after(delay_ms, self._buttonClickedHandler, None, metadata)
            self._button_pressed_counter += 1

    def _buttonReleasedHandler(self, event, metadata):
        self._is_button_pressed = False

    def _onShow(self):
        pass

    def getPidParameterEntry(self, motor_idx, entry_idx):
        return int(self.pid_entries[motor_idx][entry_idx].get())

    def setPidParameterEntry(self, motor_idx, entry_idx, value):
        self.pid_entries[motor_idx][entry_idx].delete(0, 'end')
        self.pid_entries[motor_idx][entry_idx].insert(0, value)

    def getPidParameters(self, motor_idx):
        pid_params = []
        for entry in self.pid_entries[motor_idx]:
            pid_params.append(int(entry.get()))
        return pid_params

    def getPidKp(self, motor_idx):
        return self.getPidParameterEntry(motor_idx, 0)

    def getPidKi(self, motor_idx):
        return self.getPidParameterEntry(motor_idx, 1)

    def getPidKd(self, motor_idx):
        return self.getPidParameterEntry(motor_idx, 2)

    def getVelocity(self, motor_idx):
        return int(self.velocity_entries[motor_idx].get())

class PagePathPlannerControl(MotorControlPage):
    def __init__(self, root, controller):
        MotorControlPage.__init__(self)
        self._controller = controller
        self._root = root

        self.robotx = 0
        self.roboty = 0
        self.robottheta = 0

        self.goalx = 0
        self.goaly = 0

        self.pathx = []
        self.pathy = []

        self.obstaclesx = []
        self.obstaclesy = []

        frame_graph_and_buttons = Tk.Frame(self)
        frame_graph_and_buttons.pack()

        frame_graph_width = 900;
        frame_graph_height = 350;
        frame_graph = Tk.Frame(frame_graph_and_buttons, width = frame_graph_width, height = frame_graph_height, borderwidth = 3, relief = 'groove')
        frame_graph.pack(side = Tk.LEFT)

        buttons_graph_width = 200;
        buttons_graph_height = frame_graph_height
        frame_buttons = Tk.Frame(frame_graph_and_buttons, borderwidth = 3, width = buttons_graph_width, height = buttons_graph_height, relief = 'groove')
        frame_buttons.pack(side = Tk.LEFT)
        frame_buttons.pack_propagate(False)

        buttons_configs = [("Zoom+", "zoomin"), ("Zoom-", "zoomout"), ("GO!", "go"), ("STOP", "stop")]
        for button_text, button_metadata in buttons_configs:
            button = Tk.Button(frame_buttons, text = button_text, width = 15, borderwidth = 3, relief = 'raised')
            button.bind('<Button-1>', lambda event, metadata = button_metadata: self._buttonClickedHandler(event, metadata))
            button.pack(side = Tk.TOP, pady = 10)

        root.update_idletasks()

        dpi = 100
        self.fig = plt.figure(figsize=(frame_graph['width'] / float(dpi), frame_graph['height'] / float(dpi)), dpi = dpi)
        plotcanvas = FigureCanvasTkAgg(self.fig, frame_graph)
        plotcanvas.get_tk_widget().pack()
        self.ax = self.fig.gca()
        self.ax.set_xlim(-5.0, 5.0)
        self.ax.set_ylim(-5.0, 5.0)
        self.ax.set_aspect('equal', adjustable = 'box')
        self.plot_path, = self.ax.plot([], [], ".r", markersize = 3)
        self.plot_bounds, = self.ax.plot([], [], ".g", markersize = 20)
        self.plot_goal, = self.ax.plot([], [], "*r", markersize = 10)
        self.plot_robot, = self.ax.plot([], [], "sb", markersize = 6)
        self.plot_sensor_circle = plt.Circle((self.robotx, self.roboty), 100, fill = False)
        self.ax.add_artist(self.plot_sensor_circle)
        self.plot_obst_circles = []

        self.fig.canvas.mpl_connect('button_press_event', lambda event: self._buttonClickedHandler(event, "canvasclick"))
        self.ani = animation.FuncAnimation(self.fig, self._updatePlot, interval = 10, blit = True)

    def _buttonClickedHandler(self, event, metadata):
        if metadata == "canvasclick":
            self.setGoalPosition(event.xdata, event.ydata)
            self._controller.setGoalPosition(event.xdata, event.ydata)
        elif metadata == "zoomin":
            self.changeLimitsBy(-1.0)
        elif metadata == "zoomout":
            self.changeLimitsBy(1.0)
        elif metadata == "go":
            self._controller.startPathPlanning()
        elif metadata == "stop":
            self._controller.stopPathPlanning()

    def _updatePlot(self, i):
        self.plot_path.set_xdata(self.pathx)
        self.plot_path.set_ydata(self.pathy)

        self.plot_bounds.set_xdata(self.obstaclesx)
        self.plot_bounds.set_ydata(self.obstaclesy)

        self.plot_goal.set_xdata(self.goalx)
        self.plot_goal.set_ydata(self.goaly)

        self.plot_robot.set_xdata(self.robotx)
        self.plot_robot.set_ydata(self.roboty)

        self.ax.xaxis.set_major_locator(ticker.AutoLocator())
        self.ax.yaxis.set_major_locator(ticker.AutoLocator())
        self.ax.grid(which = 'major', alpha = 0.5)

        self.plot_sensor_circle.center = (self.robotx, self.roboty)

        return [self.plot_path,
                self.plot_bounds,
                self.plot_goal,
                self.plot_robot,
                self.plot_sensor_circle,
                *self.plot_obst_circles]

    def changeLimitsBy(self, value):
        xmin, xmax, = self.ax.get_xlim()
        self.ax.set_xlim(xmin - value, xmax + value)

        ymin, ymax, = self.ax.get_ylim()
        self.ax.set_ylim(ymin - value, ymax + value)

        self.fig.canvas.draw()

    def setPath(self, pathx, pathy):
        self.pathx = pathx
        self.pathy = pathy

    def getGoalPosition(self):
        return self.goalx, self.goaly

    def setGoalPosition(self, x, y):
        self.goalx = x
        self.goaly = y

    def getRobotPosition(self):
        return self.robotx, self.roboty

    def setRobotPosition(self, x, y):
        self.robotx = x
        self.roboty = y

    def getRobotOrientation(self):
        return self.robottheta

    def setRobotOrientation(self, theta):
        self.robottheta = theta

    def _onShow(self):
        pass

class View:
    def __init__(self, controller):
        self._controller = controller
        self._is_button_pressed = False

        self.root = Tk.Tk()
        self.root.title("RoboMecanum Control Panel")
        self.root.attributes('-zoomed', True)
        self.root.wm_minsize(width = 700, height = 650)
        self.root.protocol('WM_DELETE_WINDOW', self._windowExitHandler)

        self.small_font = Tk.font.Font(family = 'Consolas', size = 10)
        self.title_font = Tk.font.Font(family = 'Consolas', size = 18, weight = 'bold')

        frame_title = Tk.Frame(self.root, height = 50, borderwidth = 3, relief = 'raised')
        frame_motor = Tk.Frame(self.root, borderwidth = 3, relief = 'sunken')
        frame_motor_control = Tk.Frame(frame_motor, borderwidth = 3, relief = 'groove')
        frame_motor_graphs = Tk.Frame(frame_motor, borderwidth = 3, relief = 'groove')

        frame_graph_width = 1100;
        frame_graph_height = 350;
        frame_graph = Tk.Frame(frame_motor_graphs, width = frame_graph_width, height = frame_graph_height, borderwidth = 3, relief = 'groove')

        frame_title.grid(row = 0, column = 0, sticky = 'ew')
        frame_motor.grid(row = 1, column = 0, sticky = 'nsew')
        self.root.rowconfigure(index = 0, weight = 0)
        self.root.rowconfigure(index = 1, weight = 1)
        self.root.columnconfigure(index = 0, weight = 1)

        frame_motor_control.grid(row = 0, column = 0, sticky = 'nsew')
        frame_motor_graphs.grid(row = 1, column = 0, sticky = 'ew')
        frame_motor.rowconfigure(index = 0, weight = 1)
        frame_motor.rowconfigure(index = 1, weight = 0)
        frame_motor.columnconfigure(index = 0, weight = 1)

        frame_graph.grid(row = 0, column = 0)
        frame_motor_graphs.columnconfigure(index = 0, weight = 1)
        frame_motor_graphs.rowconfigure(index = 0, weight = 1)

        title_label = Tk.Label(frame_title, text = "RoboMecanum Control Panel", font = self.title_font)
        pid_control_button = Tk.Button(frame_title, text = "PID Direct Control", font = self.title_font, borderwidth = 3, relief = 'raised')
        pid_control_button.bind('<Button-1>', lambda event: self._buttonClickedHandler(event, "pid_control"))
        velocity_vector_button = Tk.Button(frame_title, text = "Velocity Vector Control", font = self.title_font, borderwidth = 3, relief = 'raised')
        velocity_vector_button.bind('<Button-1>', lambda event: self._buttonClickedHandler(event, "velocity_vector"))
        path_control_button = Tk.Button(frame_title, text = "Path Planning", font = self.title_font, borderwidth = 3, relief = 'raised')
        path_control_button.bind('<Button-1>', lambda event: self._buttonClickedHandler(event, "path_planning"))

        title_label.pack(fill = Tk.X, side = Tk.LEFT, expand = True)
        pid_control_button.pack(fill = Tk.X, side = Tk.LEFT, expand = True)
        velocity_vector_button.pack(fill = Tk.X, side = Tk.LEFT, expand = True)
        path_control_button.pack(fill = Tk.X, side = Tk.LEFT, expand = True)

        self._direct_pid_control = PagePidDirectControl(controller)
        self._velocity_vector_control = PageVelocityVectorControl(self.root, controller)
        self._path_planning_control = PagePathPlannerControl(self.root, controller)

        self._velocity_vector_control.place(in_= frame_motor_control, x = 0, y = 0, relwidth = 1, relheight = 1)
        self._direct_pid_control.place(in_= frame_motor_control, x = 0, y = 0, relwidth = 1, relheight = 1)
        self._path_planning_control.place(in_= frame_motor_control, x = 0, y = 0, relwidth = 1, relheight = 1)

        self.motors_graph = MotorsFrameGraph(frame_graph)

        self._velocity_vector_control.show()

    def _buttonClickedHandler(self, event, metadata):
        if metadata == "pid_control":
            self._direct_pid_control.show()
        elif metadata == "velocity_vector":
            self._velocity_vector_control.show()
        elif metadata == "path_planning":
            self._path_planning_control.show()

    def _windowExitHandler(self):
        self._controller.windowExitHandler()

    def getPidKp(self, motor_idx):
        return self._direct_pid_control.getPidKp(motor_idx)

    def getPidKi(self, motor_idx):
        return self._direct_pid_control.getPidKi(motor_idx)

    def getPidKd(self, motor_idx):
        return self._direct_pid_control.getPidKd(motor_idx)

    def getVelocity(self, motor_idx):
        return self._direct_pid_control.getVelocity(motor_idx)

    def getRobotPosition(self):
        return self._path_planning_control.getRobotPosition()

    def setRobotPosition(self, x, y):
        self._path_planning_control.setRobotPosition(x, y)

    def getRobotOrientation(self):
        return self._path_planning_control.getRobotOrientation()

    def setRobotOrientation(self, theta):
        self._path_planning_control.setRobotOrientation(theta)

    def setRobotPath(self, pathx, pathy):
        self._path_planning_control.setPath(pathx, pathy)

    def updateFrontRightMotorSpeed(self, value):
        self.motors_graph.setSpeed(0, value)

    def updateFrontLeftMotorSpeed(self, value):
        self.motors_graph.setSpeed(1, value)

    def updateRearRightMotorSpeed(self, value):
        self.motors_graph.setSpeed(2, value)

    def updateRearLeftMotorSpeed(self, value):
        self.motors_graph.setSpeed(3, value)

    def stop(self):
        plt.close("all")
        self.root.destroy()

    def start(self):
        self.root.deiconify()
        self.root.mainloop()


class Controller:

    ROBOT_MAX_SPEED = 0.5  # [m/s]
    ROBOT_ENC_IMP_PER_REV = 4480 # [imp]
    ROBOT_LX = 0.115 # [m]
    ROBOT_LY = 0.14  # [m]
    ROBOT_R  = 0.06  # [m]
    ROBOT_SIZE_RADIUS = 0.2

    ROBOT_DT = 0.03125

    ROBOT_WHEEL_ALPHA = np.deg2rad(45.0)
    ROBOT_WHEEL_BETA = 1.0

    CMD_SETPOINTS = "sp={}={}={}={}\n"
    CMD_SETPOINT = "setpoint_{}={}\n"
    CMD_SETKP = "pidkp_{}={}\n"
    CMD_SETKI = "pidki_{}={}\n"
    CMD_SETKD = "pidkd_{}={}\n"

    INFO_SPEED = "Speed"

    IDX_MOTOR_FR = 0
    IDX_MOTOR_FL = 1
    IDX_MOTOR_RR = 2
    IDX_MOTOR_RL = 3
    MOTOR_INDEXES = [IDX_MOTOR_FR, IDX_MOTOR_FL, IDX_MOTOR_RR, IDX_MOTOR_RL]

    DESC_MOTOR_FR = "FR"
    DESC_MOTOR_FL = "FL"
    DESC_MOTOR_RR = "RR"
    DESC_MOTOR_RL = "RL"
    MOTOR_DESC = [DESC_MOTOR_FR, DESC_MOTOR_FL, DESC_MOTOR_RR, DESC_MOTOR_RL]

    def __init__(self):
        self.PanelView = View(self)
        self.PanelModel = Model(self)
        self.PanelView.setRobotOrientation(np.deg2rad(90))

        self.PanelModel.path_planner_observable.addObserverCallback(self.PanelView.setRobotPath)

    def incrementPidParamEntryHandler(self, entry_idx):
        current_val = self.PanelView.getPidParameterEntry(entry_idx)
        updated_val = current_val + 1
        self.PanelView.setPidParameterEntry(entry_idx, updated_val)

    def decrementPidParamEntryHandler(self, entry_idx):
        current_val = self.PanelView.getPidParameterEntry(entry_idx)
        updated_val = current_val - 1
        self.PanelView.setPidParameterEntry(entry_idx, updated_val)

    def newUartDataHandler(self, line):
        if line.startswith(self.INFO_SPEED):
            try:
                speed_values = list(map(int, line.rstrip().split("=")[1:]))
                self.PanelView.updateFrontRightMotorSpeed(speed_values[self.IDX_MOTOR_FR])
                self.PanelView.updateFrontLeftMotorSpeed(speed_values[self.IDX_MOTOR_FL])
                self.PanelView.updateRearRightMotorSpeed(speed_values[self.IDX_MOTOR_RR])
                self.PanelView.updateRearLeftMotorSpeed(speed_values[self.IDX_MOTOR_RL])
                self.setNewRobotPositionGivenWheelDistances(speed_values, self.ROBOT_DT)
            except ValueError:
                print("Cannot convert speed to value: {}".format(line.replace('\n', "\\n")))
        else:
            print("Controller got new uart line: >>{}<<".format(line.replace('\n', "\\n")))

    def convertEncoderImpulsesToMeters(self, enc_imp):
        return (enc_imp / self.ROBOT_ENC_IMP_PER_REV) * 2 * np.pi * self.ROBOT_R

    def translateRobotVelocitiesToGlobalVelocities(self, vx, vy, omega, theta):
        # rotate counter clockwise by theta
        vx_trans = vx * np.cos(theta) - vy * np.sin(theta)
        vy_trans = vx * np.sin(theta) + vy * np.cos(theta)

        omega_trans = omega

        return vx_trans, vy_trans, omega_trans

    def translateGlobalVelocitiesToLocalVelocities(self, vx, vy, omega, theta):
        pass

    def setNewRobotPositionGivenRobotVelocities(self, vx, vy, omega, dt):
        vx, vy, omega = [self.convertEncoderImpulsesToMeters(vel_imp_per_sec) for vel_imp_per_sec in [vx, vy, omega]]

        current_x, current_y = self.PanelView.getRobotPosition()
        current_theta = self.PanelView.getRobotOrientation()

        vx, vy, omega = self.translateRobotVelocitiesToGlobalVelocities(vx, vy, omega, current_theta)

        current_x += vx * dt
        current_y += vy * dt
        current_theta += omega * dt

        self.PanelModel.setStartPosition(current_x, current_y)
        self.PanelView.setRobotPosition(current_x, current_y)
        self.PanelView.setRobotOrientation(current_theta)

    def setNewRobotPositionGivenWheelDistances(self, given_wheel_positions, dt):
        d1, d2, d3, d4 = given_wheel_positions
        vx = 0.25 * (d1 + d2 + d3 + d4)
        vy = 0.25 * (d1 - d2 - d3 + d4) * np.tan(self.ROBOT_WHEEL_ALPHA)
        omega = (d1 - d2 + d3 - d4) * self.ROBOT_WHEEL_BETA
        self.setNewRobotPositionGivenRobotVelocities(vx, vy, omega, dt)

    def calculateInverseKinematics(self, desired_speeds):
        jacobian = np.array([
                             [1,  1,  (self.ROBOT_LX + self.ROBOT_LY)],
                             [1, -1, -(self.ROBOT_LX + self.ROBOT_LY)],
                             [1, -1,  (self.ROBOT_LX + self.ROBOT_LY)],
                             [1,  1, -(self.ROBOT_LX + self.ROBOT_LY)]
                            ])
        omegas_rad_per_sec = (1 / self.ROBOT_R) * jacobian.dot(desired_speeds)
        omegas_enc_per_sec = ((omegas_rad_per_sec / (2 * np.pi)) * self.ROBOT_ENC_IMP_PER_REV)
        omegas_enc_per_sec = omegas_enc_per_sec.astype(int)
        return omegas_enc_per_sec

    def setVelocityVector(self, angle_rad, amount_0_to_100):
        desired_speed_m_per_sec = (amount_0_to_100 / 100) * self.ROBOT_MAX_SPEED
        vx = desired_speed_m_per_sec * np.cos(angle_rad)
        vy = desired_speed_m_per_sec * np.sin(angle_rad)
        omega = 0
        velocities = self.calculateInverseKinematics([vx, vy, omega])

        #self.PanelModel.sendStringOverUart(self.CMD_SETPOINTS.format(*velocities))
        for velocity, desc in zip(velocities, self.MOTOR_DESC):
            self.PanelModel.sendStringOverUart(self.CMD_SETPOINT.format(desc, velocity))

    def setPidPoint(self):
        for idx, desc in zip(self.MOTOR_INDEXES, self.MOTOR_DESC):
            kp = self.PanelView.getPidKp(idx)
            ki = self.PanelView.getPidKi(idx)
            kd = self.PanelView.getPidKd(idx)
            velocity = self.PanelView.getVelocity(idx)
            self.PanelModel.sendStringOverUart(self.CMD_SETKP.format(desc, kp))
            self.PanelModel.sendStringOverUart(self.CMD_SETKI.format(desc, ki))
            self.PanelModel.sendStringOverUart(self.CMD_SETKD.format(desc, kd))
            self.PanelModel.sendStringOverUart(self.CMD_SETPOINT.format(desc, velocity))

    def setGoalPosition(self, gx, gy):
        self.PanelModel.setGoalPosition(gx, gy)

    def startPathPlanning(self):
        self.PanelModel.startPathPlanning()

    def stopPathPlanning(self):
        pass

    def windowExitHandler(self):
        self.PanelView.stop()
        self.PanelModel.stop()

    def run(self):
        self.PanelView.start()


if __name__ == '__main__':
    PanelController = Controller()
    PanelController.run()

