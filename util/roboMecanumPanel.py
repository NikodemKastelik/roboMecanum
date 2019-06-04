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
import random
import threading
import queue
import time
import math
import multiprocessing
import os
import pty
import fcntl
import csv

class PointMap:
    def __init__(self, reso, obstacles_observable):
        self._obstacles_observable = obstacles_observable
        self._reso = reso
        self._static_obstacles_x = []
        self._static_obstacles_y = []
        self._static_obstacles_corners = np.array([
                                                  [2.9,  0.0],
                                                  [2.9,  3.1],
                                                  [2.9,  3.1],
                                                  [2.9,  3.21],
                                                  [0.7,  3.21],
                                                  [0.7,  2.0],
                                                  [0.0,  2.0],
                                                  [0.0,  4.17],
                                                  [1.85, 4.17],
                                                  [1.85, 4.87],
                                                  [1.6,  4.87],
                                                  [1.6,  6.07],
                                                  [2.8,  6.07],
                                                  [2.8,  4.87],
                                                  [2.55, 4.87],
                                                  [2.55, 4.17],
                                                  [2.70, 4.17],
                                                  [2.70, 4.07],
                                                  [2.83, 4.07],
                                                  [2.83, 4.17],
                                                  [4.13, 4.17],
                                                  [4.13, 3.21],
                                                  [3.71, 3.21],
                                                  [3.71, 3.1],
                                                  [4.25, 3.1],
                                                  [4.25, 0.0],
                                                  [2.9,  0.0]
                                                  ])
        self._dynamic_obstacles_x = []
        self._dynamic_obstacles_y = []

    def _generateInterPoints(self):
        interxy = []
        for idx in range(len(self._static_obstacles_corners) - 1):
            x1, y1 = self._static_obstacles_corners[idx]
            x2, y2 = self._static_obstacles_corners[idx + 1]

            dx = x2 - x1
            dy = y2 - y1
            d = np.hypot(dx, dy)
            angle = np.arctan2(dy, dx)
            ddx = np.cos(angle) * self._reso
            ddy = np.sin(angle) * self._reso
            for index in range(len(np.arange(0, d, self._reso))):
                px = x1 + index * ddx
                py = y1 + index * ddy
                interxy.append([px, py])

        pointsxy = np.append(self._static_obstacles_corners, interxy, axis = 0)

        self._static_obstacles_x = pointsxy[:, 0].tolist()
        self._static_obstacles_y = pointsxy[:, 1].tolist()

    def initializeKnownObstacles(self):
        self._generateInterPoints()
        self._obstacles_observable.fireCallbacks(self._static_obstacles_x, self._static_obstacles_y)

    def getPoints(self):
        return self._static_obstacles_x + self._dynamic_obstacles_x, \
               self._static_obstacles_y + self._dynamic_obstacles_y

    def filterPoint(self, sx, sy, newpx, newpy):
        newp_vect_mag = np.hypot(newpx - sx, newpy - sy)
        newp_vect_angle = np.arctan2(newpy - sy, newpx - sx)
        angle_reso = np.arctan2(self._reso, newp_vect_mag)
        for idx, (px, py) in enumerate(zip(self._dynamic_obstacles_x, self._dynamic_obstacles_y)):
            p_vect_mag = np.hypot(px - sx, py - sy)
            p_vect_angle = np.arctan2(py - sy, px - sx)
            if newp_vect_mag > p_vect_mag and abs(p_vect_angle - newp_vect_angle) < angle_reso:
                del self._dynamic_obstacles_x[idx]
                del self._dynamic_obstacles_y[idx]
        self._obstacles_observable.fireCallbacks(*self.getPoints())

    def addPoint(self, newpx, newpy):
        for px, py in zip(self._dynamic_obstacles_x, self._dynamic_obstacles_y):
            dx = newpx - px
            dy = newpy - py
            if np.hypot(dx, dy) < self._reso:
                return

        self._dynamic_obstacles_x.append(newpx)
        self._dynamic_obstacles_y.append(newpy)
        self._obstacles_observable.fireCallbacks(*self.getPoints())

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
        self._createVirtualBoundaries(-20.0, -20.0, 20.0, 20.0)

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
        motion = [[1, 0, 0.8],
                  [0, 1, 0.8],
                  [-1, 0, 0.8],
                  [0, -1, 0.8],
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
                    print("Unreachable point")
                    return [], []
                else:
                    print(str(e))
                    return [], []
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
       return np.sqrt((px - gx) ** 2 + (py - gy) ** 2) <= self.reso

class PathPlanner:
    def __init__(self,
                 position_observable,
                 path_observable,
                 algorithm):
        self._position_observable = position_observable
        self._path_observable = path_observable
        self._algo = algorithm
        self._sx = 0
        self._sy = 0
        self._gx = 0
        self._gy = 0
        self._pathx = []
        self._pathy = []
        self._obx = []
        self._oby = []
        self._running = False
        self._algo_thread = None

    def _algorithmLoopProcess(self, flag_running, pipe_to_process):
        while flag_running.is_set():
            if pipe_to_process.poll():
                algo_data = pipe_to_process.recv()
                sx, sy, gx, gy, obx, oby = algo_data

                time_start = time.perf_counter()
                pathx, pathy = self._algo.planPath(sx, sy,
                                                   gx, gy,
                                                   obx, oby)
                print("Astar took: {} [s]".format(time.perf_counter() - time_start))

                pipe_to_process.send((pathx, pathy))
            time.sleep(0.001)

    def _algorithmLoop(self):
        flag_running = multiprocessing.Event()
        flag_running.set()
        pipe_to_process, pipe_from_process = multiprocessing.Pipe()
        algo_process = multiprocessing.Process(target = self._algorithmLoopProcess,
                                               args=(flag_running, pipe_to_process,))
        algo_process.start()
        while self._running:
            time_start = time.perf_counter()
            pipe_from_process.send((self._sx, self._sy,
                                    self._gx, self._gy,
                                    self._obx, self._oby))
            self._pathx, self._pathy = pipe_from_process.recv()
            self._path_observable.fireCallbacks(self._pathx, self._pathy)
            time.sleep(0.01)
        flag_running.clear()
        algo_process.join()

    def getStartPosition(self):
        return self._sx, self._sy

    def setStartPosition(self, sx, sy):
        self._sx = sx
        self._sy = sy
        self._position_observable.fireCallbacks(sx, sy)

    def getGoalPosition(self):
        return self._gx, self._gy

    def setGoalPosition(self, gx, gy):
        self._gx = gx
        self._gy = gy

    def getObstacles(self):
        return self._obx, self._oby

    def setObstacles(self, obx, oby):
        self._obx = obx
        self._oby = oby

    def getPlannedPath(self):
        return self._pathx, self._pathy

    def isGoalAchieved(self):
        return self._algo.isGoalAchieved(self._sx, self._sy,
                                         self._gx, self._gy)

    def start(self):
        if not self._running:
            self._running = True
            self._algo_thread = threading.Thread(target = self._algorithmLoop)
            self._algo_thread.start()

    def stop(self):
        if self._running:
            self._running = False
            self._algo_thread.join()


class UltrasonicRadar:
    RADAR_STEPS_PER_REV = 4096
    RADAR_POSITION_UPDATE_TIMEOUT = 0.2
    RADAR_DISTANCE_MEASUREMENT_TIMEOUT = 0.1
    RADAR_DISTANCE_CM_RELIABLE = 100
    CMD_SET_POSITION = "stepper={}\n"
    CMD_SENSE_DISTANCE = "dist_measure\n"

    def __init__(self, sensing_cone_angle_rad, data_handler):
        self._data_handler = data_handler
        self._sensing_cone_angle_steps = self.angleToSteps(sensing_cone_angle_rad)
        self._facing_angle = 0
        self._current_position = 0
        self._running = False
        self._radar_rotation_thread = threading.Thread(target = self._radarRotationLoop)

    def _radarRotationLoop(self):
        angle = self._facing_angle
        desired_position = int(self._sensing_cone_angle_steps / 2)
        self._data_handler(self.CMD_SET_POSITION.format(desired_position))
        position_update_time = time.perf_counter()
        distance_update_time = time.perf_counter()
        while self._running:
            new_position = None
            if desired_position == self._current_position:
                if desired_position > self.angleToSteps(angle):
                    new_position = desired_position - self._sensing_cone_angle_steps
                else:
                    new_position = desired_position + self._sensing_cone_angle_steps

            if self._facing_angle > angle:
                angle = self._facing_angle
                new_position = self.angleToSteps(angle) + self._sensing_cone_angle_steps / 2
            if self._facing_angle < angle:
                angle = self._facing_angle
                new_position = self.angleToSteps(angle) - self._sensing_cone_angle_steps / 2

            if new_position is not None:
                desired_position = int(new_position)

            if time.perf_counter() > position_update_time + self.RADAR_POSITION_UPDATE_TIMEOUT:
                position_update_time = time.perf_counter()
                self._data_handler(self.CMD_SET_POSITION.format(desired_position))

            if time.perf_counter() > distance_update_time + self.RADAR_DISTANCE_MEASUREMENT_TIMEOUT:
                distance_update_time = time.perf_counter()
                self._data_handler(self.CMD_SENSE_DISTANCE)

            time.sleep(0.05)

    def resetPosition(self):
        self._data_handler(self.CMD_SET_POSITION.format(0))

    def angleToSteps(self, angle_rad):
        return int((angle_rad / (2 * np.pi)) * self.RADAR_STEPS_PER_REV)

    def stepsToAngle(self, steps):
        return (steps / self.RADAR_STEPS_PER_REV) * (2 * np.pi)

    def setFacingAngle(self, angle_rad):
        self._facing_angle = angle_rad * (-1)

    def notifyPosition(self, new_position, *args):
        self._current_position = new_position

    def start(self):
        if not self._running:
            self._running = True
            self._radar_rotation_thread.start()

    def stop(self):
        if self._running:
            self._running = False
            self._radar_rotation_thread.join()


class MotorRecorder:
    def __init__(self, filename, dt):
        self._filename = filename
        self._csv_writer = None
        self._dt = dt

    def startRecording(self, labels):
        if not self.isRecording():
            self._fd = open(self._filename, "w+")
            self._csv_writer = csv.writer(self._fd)
            self._csv_writer.writerow(["Timestamp", *labels])
            self._timestamp = 0

    def stopRecording(self):
        if self.isRecording():
            self._csv_writer = None
            self._fd.close()
            self._timestamp = 0

    def isRecording(self):
        if self._csv_writer == None:
            return False
        else:
            return True

    def record(self, values):
        if self.isRecording():
            self._csv_writer.writerow([self._timestamp, *values])
            self._timestamp += self._dt


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
    ROBOT_MAX_SPEED = 0.5  # [m/s]
    ROBOT_ENC_IMP_PER_REV = 4480 # [imp]
    ROBOT_LX = 0.115 # [m]
    ROBOT_LY = 0.14  # [m]
    ROBOT_R  = 0.03  # [m]
    ROBOT_SIZE_RADIUS = 0.2
    ROBOT_SET_SPEED_MIN = 600

    ROBOT_DT = 0.03125

    ROBOT_WHEEL_ALPHA = np.deg2rad(45.0)
    ROBOT_WHEEL_BETA = 1.0

    CMD_SETPOINTS = "sp={}={}={}={}\n"
    CMD_SETPOINT = "setpoint_{}={}\n"
    CMD_SETKP = "pidkp_{}={}\n"
    CMD_SETKI = "pidki_{}={}\n"
    CMD_SETKD = "pidkd_{}={}\n"
    CMD_SETPWM = "pwm={}\n"

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

    INFO_SPEED = "Speed"
    INFO_DIST_MEASURE = "Distance="

    ZIGBEE_MAX_BYTES_PER_ACCESS = 46
    ZIGBEE_DELAY_BETWEEN_ACCESS = 0.02
    ZIGBEE_PREFIX               = "P2P FB31 "

    RECORDING_FILENAME = "readout.csv"

    def __init__(self):
        self.wheel_speeds_observable = Observable()
        self.robot_position_observable = Observable()
        self.robot_path_observable = Observable()
        self.obstacles_observable = Observable()
        self.radar_observable = Observable()

        #self._model_running = True
        #master, slave = pty.openpty()
        #self._dummy_serial_thread = threading.Thread(target = self._dummySerialLoop, args = (master,))
        #self._dummy_serial_thread.start()
        #self._uart_mngr = SerialDeviceManager(os.ttyname(slave))

        self._model_running = False
        self._uart_mngr = SerialDeviceManager("/dev/ttyUSB0")

        self._obstacle_map = PointMap(self.ROBOT_SIZE_RADIUS, self.obstacles_observable)

        self._radar = UltrasonicRadar(np.deg2rad(360), self.sendStringOverUart)

        self._uart_mngr.setBytesSendPerAccess(self.ZIGBEE_MAX_BYTES_PER_ACCESS)
        self._uart_mngr.setPerAccessDelayMs(self.ZIGBEE_DELAY_BETWEEN_ACCESS)
        self._uart_mngr.setPrefix(self.ZIGBEE_PREFIX)

        self._uart_mngr_reader_loop_thread = threading.Thread(target = self._uartMngrReaderLoop)

        self._path_algo = AStarAlgorithm(reso = 0.05, safedist = self.ROBOT_SIZE_RADIUS * 1.00)
        self._path_planner = PathPlanner(self.robot_position_observable,
                                         self.robot_path_observable,
                                         self._path_algo)

        self._motor_recorder = MotorRecorder(self.RECORDING_FILENAME, self.ROBOT_DT)

        self.wheel_speeds_observable.addObserverCallback(self._motor_recorder.record)
        self.obstacles_observable.addObserverCallback(self._path_planner.setObstacles)
        self.robot_path_observable.addObserverCallback(self._moveAlongPath)
        self.radar_observable.addObserverCallback(self._radar.notifyPosition)
        self.radar_observable.addObserverCallback(self._calculateObstacleAbsolutePosition)

    def _dummySerialLoop(self, fd):
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
        last_speeds = (0, 0, 0 ,0)
        radar_position = 0
        while self._model_running:
            try:
                readout = os.read(fd, 1024).decode()
                for line in readout.splitlines():
                    command = line.split(" ")[-1]
                    if command.startswith("sp="):
                        last_speeds = list(map(int, command.split("=")[1:]))
                    elif command.startswith("stepper="):
                        radar_position = int(command.split("=")[-1])
                    elif command.startswith("dist_measure"):
                        distance = random.randint(100, 200)
                        os.write(fd, "Distance={}={}\n".format(radar_position, distance).encode())
            except Exception as e:
                if str(e) == "[Errno 11] Resource temporarily unavailable":
                    pass
                else:
                    raise e

            try:
                os.write(fd, "Speed={}={}={}={}\n".format(*last_speeds).encode())
            except Exception as e:
                if str(e) == "[Errno 11] Resource temporarily unavailable":
                    pass
                else:
                    raise e

            time.sleep(0.01)

    def _uartMngrReaderLoop(self):
        self._uart_mngr.start()
        while self._model_running:
            line = self._uart_mngr.recv()
            if line:
                self._newUartDataHandler(line)
            time.sleep(0.001)

    def _newUartDataHandler(self, line):
        if line.startswith(self.INFO_SPEED):
            try:
                speed_values = list(map(int, line.rstrip().split("=")[1:]))
                self._setNewRobotPositionGivenWheelDistances(speed_values)
                self.wheel_speeds_observable.fireCallbacks(speed_values)
            except ValueError:
                print("Cannot convert speed to value: {}".format(line.replace('\n', "\\n")))

        elif line.startswith(self.INFO_DIST_MEASURE):
            try:
                position, distance_cm = list(map(int, line.rstrip().split("=")[1:]))
                self.radar_observable.fireCallbacks(position, distance_cm)
                print("Distance: {} at {}".format(distance_cm, position))
            except ValueError:
                print("Cannot convert distance measurement to value: {}".format(line.replace('\n', "\\n")))

        else:
            print("Model got new uart line: >>{}<<".format(line.replace('\n', "\\n")))

    def _calculateObstacleAbsolutePosition(self, sensor_angle_steps, distance_cm):
        sx, sy = self._path_planner.getStartPosition()
        theta = self._radar.stepsToAngle(sensor_angle_steps) * (-1) + self._orientation
        distance_m = distance_cm / 100
        dx = distance_m * np.cos(theta)
        dy = distance_m * np.sin(theta)
        newpx = float(sx + dx)
        newpy = float(sy + dy)
        self._obstacle_map.filterPoint(sx, sy, newpx, newpy)
        if distance_cm < self._radar.RADAR_DISTANCE_CM_RELIABLE:
            self._obstacle_map.addPoint(newpx, newpy)

    def _convertEncoderImpulsesToMeters(self, enc_imp):
        return (enc_imp / self.ROBOT_ENC_IMP_PER_REV) * 2 * np.pi * self.ROBOT_R

    def _translateRobotVelocitiesToGlobalVelocities(self, vx, vy, omega, theta):
        # rotate counter clockwise by theta
        vx_trans = vx * np.cos(theta) - vy * np.sin(theta)
        vy_trans = vx * np.sin(theta) + vy * np.cos(theta)

        omega_trans = omega

        return vx_trans, vy_trans, omega_trans

    def _translateGlobalVelocitiesToLocalVelocities(self, vx, vy, omega, theta):
        pass

    def _setNewRobotPositionGivenRobotVelocities(self, vx, vy, omega, dt):
        vx, vy, omega = [self._convertEncoderImpulsesToMeters(vel_imp_per_sec) for vel_imp_per_sec in [vx, vy, omega]]

        current_x, current_y = self._path_planner.getStartPosition()
        current_theta = self._orientation

        vx, vy, omega = self._translateRobotVelocitiesToGlobalVelocities(vx, vy, omega, current_theta)

        current_x += vx * dt
        current_y += vy * dt
        current_theta += omega * dt

        self._orientation = current_theta
        self._path_planner.setStartPosition(current_x, current_y)

    def _setNewRobotPositionGivenWheelDistances(self, given_wheel_positions):
        dt = self.ROBOT_DT
        d1, d2, d3, d4 = given_wheel_positions
        vx = 0.25 * (d1 + d2 + d3 + d4)
        vy = 0.25 * (d1 - d2 - d3 + d4) * np.tan(self.ROBOT_WHEEL_ALPHA)
        omega = (d1 - d2 + d3 - d4) * self.ROBOT_WHEEL_BETA
        self._setNewRobotPositionGivenRobotVelocities(vx, vy, omega, dt)

    def _calculateInverseKinematics(self, desired_speeds):
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

    def _headTowardsPoint(self, px, py):
        sx, sy = self._path_planner.getStartPosition()
        dx = px - sx
        dy = py - sy

        vect_angle = np.arctan2(dy, dx) - self._orientation
        self.setVelocityVector(vect_angle, amount_0_to_100 = 25)

    def _moveAlongPath(self, pathx, pathy):
        if len(pathx) == 0 or self._path_planner.isGoalAchieved():
            self.sendStringOverUart(self.CMD_SETPOINTS.format(0, 0, 0, 0))
            return
        elif len(pathx) < 3:
            px1, py1 = self._path_planner.getStartPosition()
            px2, py2 = pathx[-1], pathy[-1]
        else:
            px1, py1 = pathx[-2], pathy[-2]
            px2, py2 = pathx[-3], pathy[-3]
        dx = px2 - px1
        dy = py2 - py1
        vect_angle = np.arctan2(dy, dx) - self._orientation
        self.setVelocityVector(vect_angle, amount_0_to_100 = 20)

    def setVelocityVector(self, angle_rad, amount_0_to_100):
        desired_speed_m_per_sec = (amount_0_to_100 / 100) * self.ROBOT_MAX_SPEED
        vx = desired_speed_m_per_sec * np.cos(angle_rad)
        vy = desired_speed_m_per_sec * np.sin(angle_rad)
        omega = 0
        velocities = [vi if abs(vi) > self.ROBOT_SET_SPEED_MIN else 0
                      for vi in self._calculateInverseKinematics([vx, vy, omega])]
        #self._radar.setFacingAngle(angle_rad)
        self.sendStringOverUart(self.CMD_SETPOINTS.format(*velocities))

    def setPidPoint(self, pid_params_and_velocity):
        self.sendStringOverUart(self.CMD_SETPWM.format(pid_params_and_velocity[0][3]))

    def setStartPosition(self, x, y):
        self._path_planner.setStartPosition(x, y)

    def setGoalPosition(self, x, y):
        self._path_planner.setGoalPosition(x, y)

    def startPathPlanning(self):
        self._path_planner.start()

    def stopPathPlanning(self):
        self._path_planner.stop()
        self.setVelocityVector(0, 0)

    def sendStringOverUart(self, data):
        if self._model_running:
            self._uart_mngr.send(data)

    def setInitialState(self):
        self._orientation = np.deg2rad(90.0)
        self._path_planner.setStartPosition(3.4, 1.0)
        self._obstacle_map.initializeKnownObstacles()

    def startRecording(self):
        self._motor_recorder.startRecording(self.MOTOR_DESC)

    def stopRecording(self):
        self._motor_recorder.stopRecording()

    def start(self):
        self._model_running = True
        self._uart_mngr_reader_loop_thread.start()
        self._radar.start()

    def stop(self):
        self.stopRecording()
        self.sendStringOverUart(self.CMD_SETPOINTS.format(0, 0, 0, 0))
        self._radar.stop()
        self._radar.resetPosition()
        time.sleep(0.1)
        self._model_running = False
        self._uart_mngr_reader_loop_thread.join()
        self._uart_mngr.stop()
        self._path_planner.stop()

class MotorsFrameGraph:
    def __init__(self, parent_frame):
        self.DATA_SIZE = 150
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

        self.velocity_vector = self._canvas.create_line(center_x, center_y,
                                                        center_x, center_y,
                                                        arrow = Tk.LAST,
                                                        width = 4)

        self.line_horizontal = self._canvas.create_line(center_x - oval_radius_px, center_y,
                                                        center_x + oval_radius_px, center_y)

        self.line_vertical = self._canvas.create_line(center_x, center_y - oval_radius_px,
                                                      center_x, center_y + oval_radius_px)

        for canvas_elem in [self.oval, self.velocity_vector, self.line_horizontal, self.line_vertical]:
            self._canvas.tag_bind(canvas_elem, '<Button-1>', self._clickedInCircleHandler)
            self._canvas.tag_bind(canvas_elem, '<ButtonRelease-1>', self._releasedCirceClickHandler)

    def redraw(self):
        self._root.update_idletasks()
        center_x = self._canvas.winfo_width() / 2
        center_y = self._canvas.winfo_height() / 2
        oval_radius_px = (self._canvas.winfo_height() / 2) - 10
        self._canvas.coords(self.oval, (center_x - oval_radius_px, center_y - oval_radius_px,
                                        center_x + oval_radius_px, center_y + oval_radius_px))
        self._canvas.coords(self.velocity_vector, (center_x, center_y,
                                                   center_x, center_y))
        self._canvas.coords(self.line_horizontal, (center_x - oval_radius_px, center_y,
                                                   center_x + oval_radius_px, center_y))
        self._canvas.coords(self.line_vertical, (center_x, center_y - oval_radius_px,
                                                 center_x, center_y + oval_radius_px))

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

        self._is_recording = False

        self._controller = controller
        self.small_font = Tk.font.Font(family = 'Consolas', size = 10)
        self.title_font = Tk.font.Font(family = 'Consolas', size = 18, weight = 'bold')

        column_labels =   ["Kp",  "Ki",  "Kd", "Velocity"]
        default_values =  ["600", "300", "0",  "0"]

        self.row_labels = ["FR", "FL", "RR", "RL"]

        self.pid_entries = []
        self.velocity_entries = []
        for col_idx in range(len(column_labels)):
            pid_label = Tk.Label(self, text = column_labels[col_idx], font = self.title_font)
            pid_label.grid(row = 0, column = col_idx + 1, columnspan = 1, sticky = 'nsew', padx = 3, pady = 3)

        for row_idx in range(1, len(self.row_labels) + 1):
            motor_label = Tk.Label(self, text = self.row_labels[row_idx - 1], font = self.title_font)
            motor_label.grid(row = row_idx, column = 0, sticky = 'nsew');

            pid_entries = []
            for col_idx, default_value in enumerate(default_values):
                entry = Tk.Entry(self, justify = Tk.CENTER, font = self.title_font)
                entry.grid(row = row_idx, column = col_idx + 1, columnspan = 1, sticky = 'nsew', padx = 20, pady = 3)
                entry.insert(0, default_value)

                if col_idx == len(column_labels) - 1:
                    self.velocity_entries.append(entry)
                else:
                    pid_entries.append(entry)
            self.pid_entries.append(pid_entries)

        button = Tk.Button(self, text = "GO!", font = self.title_font, borderwidth = 3, relief = 'raised')
        button.bind('<Button-1>', lambda event: self._buttonClickedHandler(event, "go"))
        button.grid(row = len(self.row_labels) + 2, column = int(len(column_labels) / 2), columnspan = 2, sticky = 'nsew', padx = 20, pady = 5)

        self._recording_button = Tk.Button(self, text = "Start Recording", font = self.title_font, borderwidth = 3, relief = 'raised')
        self._recording_button.bind('<Button-1>', lambda event: self._buttonClickedHandler(event, "recording"))
        self._recording_button.grid(row = len(self.row_labels) + 2, column = int(len(column_labels) / 2) + 2, columnspan = 1,
                                    sticky = 'nsew', padx = 20, pady = 5)

        for i in range(len(self.row_labels) + 2):
            self.rowconfigure(index = i, weight = 1, minsize = 30)

        self.columnconfigure(index = 0, weight = 1, minsize = 50)
        for i in range(1, 2 * len(column_labels) + 1):
            self.columnconfigure(index = i, weight = 1)

    def _buttonClickedHandler(self, event, metadata):
        if metadata == "go":
            motor_desc = []
            for motor_idx in range(len(self.row_labels)):
                motor_desc.append([self.getPidKp(motor_idx),
                                   self.getPidKi(motor_idx),
                                   self.getPidKd(motor_idx),
                                   self.getVelocity(motor_idx)])
            self._controller.setPidPoint(motor_desc)
        elif metadata == "recording":
            if self._is_recording:
                self._recording_button['text'] = "Start Recording"
                self._is_recording = False
                self._controller.stopRecording()
            else:
                self._recording_button['text'] = "Stop Recording"
                self._is_recording = True
                self._controller.startRecording()
        else:
            print("Unsupported metadata in _buttonClickedHandler: {}".format(metadata))

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
        self.plot_bounds, = self.ax.plot([], [], ".g", markersize = 3)
        self.plot_goal, = self.ax.plot([], [], "*r", markersize = 10)
        self.plot_robot, = self.ax.plot([], [], "sb", markersize = 6)
        self.plot_sensor_circle = plt.Circle((self.robotx, self.roboty), 0, fill = False)
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

        self.centerPlotOnRobot()

        return [self.plot_path,
                self.plot_bounds,
                self.plot_goal,
                self.plot_robot,
                self.plot_sensor_circle,
                *self.plot_obst_circles]

    def centerPlotOnRobot(self):
        xmin, xmax, = self.ax.get_xlim()
        dx = (xmax - xmin) / 2
        self.ax.set_xlim(self.robotx - dx, self.robotx + dx)

        ymin, ymax, = self.ax.get_ylim()
        dy = (ymax - ymin) / 2
        self.ax.set_ylim(self.roboty - dy, self.roboty + dy)

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

    def getRobotPose(self):
        return self.robotx, self.roboty, self.robottheta

    def setRobotPose(self, x, y, theta):
        self.robotx = x
        self.roboty = y
        self.robottheta = theta

    def getRobotPosition(self):
        return self.robotx, self.roboty

    def setRobotPosition(self, x, y):
        self.robotx = x
        self.roboty = y

    def getRobotOrientation(self):
        return self.robottheta

    def setRobotOrientation(self, theta):
        self.robottheta = theta

    def setObstacles(self, obx, oby):
        self.obstaclesx = obx
        self.obstaclesy = oby

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

        title_label = Tk.Label(frame_title, text = "RoboMecanum\nControl Panel", font = self.title_font)
        title_label.pack(fill = Tk.X, side = Tk.LEFT, expand = True)

        for text, metadata in [("PID Direct Control", "pid_control"),
                               ("Velocity Vector Control", "velocity_vector"),
                               ("Path Planning", "path_planning")]:
            button = Tk.Button(frame_title, text = text, font = self.title_font, borderwidth = 3, relief = 'raised')
            button.bind('<Button-1>', lambda event, arg = metadata: self._buttonClickedHandler(event, arg))
            button.pack(fill = Tk.X, side = Tk.LEFT, expand = True)

        self._direct_pid_control = PagePidDirectControl(controller)
        self._velocity_vector_control = PageVelocityVectorControl(self.root, controller)
        self._path_planning_control = PagePathPlannerControl(self.root, controller)

        for page in [self._direct_pid_control,
                     self._velocity_vector_control,
                     self._path_planning_control]:
            page.place(in_= frame_motor_control, x = 0, y = 0, relwidth = 1, relheight = 1)

        self.motors_graph = MotorsFrameGraph(frame_graph)

        self._velocity_vector_control.show()

    def _buttonClickedHandler(self, event, metadata):
        print(metadata)
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

    def getPidParameters(self, motor_idx):
        return self.getPidKp(motor_idx), self.getPidKi(motor_idx), self.getPidKd(motor_idx)

    def getVelocity(self, motor_idx):
        return self._direct_pid_control.getVelocity(motor_idx)

    def getRobotPose(self):
        return self._path_planning_control.getRobotPose()

    def setRobotPose(self, x, y, theta):
        self._path_planning_control.setRobotPose(x, y, theta)

    def getRobotPosition(self):
        return self._path_planning_control.getRobotPosition()

    def setRobotPosition(self, x, y):
        self._path_planning_control.setRobotPosition(x, y)

    def getRobotOrientation(self):
        return self._path_planning_control.getRobotOrientation()

    def setRobotorientation(self, theta):
        self._path_planning_control.setRobotOrientation(theta)

    def setRobotPath(self, pathx, pathy):
        self._path_planning_control.setPath(pathx, pathy)

    def setRobotObstacles(self, obx, oby):
        self._path_planning_control.setObstacles(obx, oby)

    def updateFrontRightMotorSpeed(self, value):
        self.motors_graph.setSpeed(0, value)

    def updateFrontLeftMotorSpeed(self, value):
        self.motors_graph.setSpeed(1, value)

    def updateRearRightMotorSpeed(self, value):
        self.motors_graph.setSpeed(2, value)

    def updateRearLeftMotorSpeed(self, value):
        self.motors_graph.setSpeed(3, value)

    def updateMotorSpeeds(self, values):
        for index, speed in enumerate(values):
            self.motors_graph.setSpeed(index, speed)

    def stop(self):
        plt.close("all")
        self.root.destroy()

    def start(self):
        self.root.deiconify()
        self.root.mainloop()


class Controller:
    def __init__(self):
        self.PanelView = View(self)
        self.PanelModel = Model()

        self.PanelModel.robot_path_observable.addObserverCallback(self.PanelView.setRobotPath)
        self.PanelModel.robot_position_observable.addObserverCallback(self.PanelView.setRobotPosition)
        self.PanelModel.wheel_speeds_observable.addObserverCallback(self.PanelView.updateMotorSpeeds)
        self.PanelModel.obstacles_observable.addObserverCallback(self.PanelView.setRobotObstacles)

        self.PanelModel.setInitialState()
        self.PanelModel.start()

    def incrementPidParamEntryHandler(self, entry_idx):
        current_val = self.PanelView.getPidParameterEntry(entry_idx)
        updated_val = current_val + 1
        self.PanelView.setPidParameterEntry(entry_idx, updated_val)

    def decrementPidParamEntryHandler(self, entry_idx):
        current_val = self.PanelView.getPidParameterEntry(entry_idx)
        updated_val = current_val - 1
        self.PanelView.setPidParameterEntry(entry_idx, updated_val)

    def setVelocityVector(self, angle_rad, amount_0_to_100):
        self.PanelModel.setVelocityVector(angle_rad, amount_0_to_100)

    def setPidPoint(self, pid_params_and_velocity):
        self.PanelModel.setPidPoint(pid_params_and_velocity)

    def setGoalPosition(self, gx, gy):
        self.PanelModel.setGoalPosition(gx, gy)

    def startPathPlanning(self):
        self.PanelModel.startPathPlanning()

    def stopPathPlanning(self):
        self.PanelModel.stopPathPlanning()

    def startRecording(self):
        self.PanelModel.startRecording()

    def stopRecording(self):
        self.PanelModel.stopRecording()

    def windowExitHandler(self):
        self.PanelView.stop()
        self.PanelModel.stop()

    def run(self):
        self.PanelView.start()


if __name__ == '__main__':
    PanelController = Controller()
    PanelController.run()

