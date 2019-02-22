import tkinter as Tk
from tkinter import font

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
style.use('ggplot')

import serial
import numpy as np
import threading
import queue
import time

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
        self._serial_device_loop_thread = threading.Thread(target = self._serialDeviceLoop)

    def _serialDeviceLoop(self):
        while self._running:
            try:
                bytes_to_send = self._write_queue.get(block=False).encode()
                self.write(bytes_to_send)
            except queue.Empty:
                pass
            try:
                line = self.readLine().decode()
            except UnicodeDecodeError:
                print("Cannot decode byte")
                line = ""
            if line:
                self._read_queue.put(line)

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

class Model:
    def __init__(self, controller):
        self._controller = controller
        self._uart_mngr = SerialDeviceManager("/dev/ttyUSB0")
        self._model_running = True
        self._uart_mngr_reader_loop_thread = threading.Thread(target = self._uartMngrReaderLoop)
        self._uart_mngr_reader_loop_thread.start()

    def _uartMngrReaderLoop(self):
        self._uart_mngr.start()
        while self._model_running:
            line = self._uart_mngr.recv()
            if line:
                self._controller.newUartDataHandler(line)
            time.sleep(0.001)

    def sendStringOverUart(self, data):
        if self._model_running:
            self._uart_mngr.send(data)

    def stop(self):
        self._model_running = False
        while self._uart_mngr_reader_loop_thread.is_alive():
            pass
        self._uart_mngr.stop()

class FrameGraph:
    def __init__(self, parent_frame):
        self.DATA_SIZE = 100
        self.INTERVAL_MS = 25
        self._current_speed = 0

        dpi = 70
        self.fig = plt.figure(figsize=(parent_frame['width'] / float(dpi), parent_frame['height'] / float(dpi)), dpi = dpi)
        self.ax1 = self.fig.add_subplot(1, 1, 1)
        self.ax1.set_ylim(-8000, 8000)
        self.ax1.set_xlim(0, self.DATA_SIZE * (self.INTERVAL_MS / 1000))
        self.line, = self.ax1.plot([],[], '.r-')
        self.plotdata = np.zeros([self.DATA_SIZE, 2])
        self.plotdata[:, 0] = np.linspace(-(self.DATA_SIZE) * self.INTERVAL_MS / 1000, 0, self.DATA_SIZE)

        plotcanvas = FigureCanvasTkAgg(self.fig, parent_frame)
        plotcanvas.get_tk_widget().pack()
        self.anim = animation.FuncAnimation(self.fig, self._updatePlot, interval = self.INTERVAL_MS, blit = True)

    def _updatePlot(self, i):
        t = (i * self.INTERVAL_MS) / 1000

        self.plotdata = np.append(self.plotdata, [[t, self._current_speed]], axis = 0)
        self.plotdata = np.delete(self.plotdata, (0), axis = 0)

        self.line.set_xdata(self.plotdata[:,0])
        self.line.set_ydata(self.plotdata[:,1])
        self.ax1.set_xlim(self.plotdata[0, 0], self.plotdata[self.DATA_SIZE - 1, 0])

        return self.line,

    def setSpeed(self, speed):
        self._current_speed = speed;

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

        frame_graph_width = 500;
        frame_graph_height = 180;
        frame_graph_fr = Tk.Frame(frame_motor_graphs, width = frame_graph_width, height = frame_graph_height, borderwidth = 3, relief = 'groove')
        frame_graph_fl = Tk.Frame(frame_motor_graphs, width = frame_graph_width, height = frame_graph_height, borderwidth = 3, relief = 'groove')
        frame_graph_rr = Tk.Frame(frame_motor_graphs, width = frame_graph_width, height = frame_graph_height, borderwidth = 3, relief = 'groove')
        frame_graph_rl = Tk.Frame(frame_motor_graphs, width = frame_graph_width, height = frame_graph_height, borderwidth = 3, relief = 'groove')

        frame_title.pack(fill = Tk.X, side = Tk.TOP)
        frame_motor.pack(fill = Tk.BOTH, side = Tk.TOP, expand = True)
        frame_motor_control.pack(fill = Tk.X, side = Tk.TOP, expand = False, padx = 5, pady = 5)
        frame_motor_graphs.pack(fill = Tk.BOTH, side = Tk.TOP, expand = True, padx = 5, pady = 5)

        frame_graph_fr.grid(row = 0, column = 1)
        frame_graph_fl.grid(row = 0, column = 0)
        frame_graph_rr.grid(row = 1, column = 1)
        frame_graph_rl.grid(row = 1, column = 0)
        frame_motor_graphs.columnconfigure(index = 0, weight = 1)
        frame_motor_graphs.columnconfigure(index = 1, weight = 1)
        frame_motor_graphs.rowconfigure(index = 0, weight = 1)
        frame_motor_graphs.rowconfigure(index = 1, weight = 1)

        column_labels = ["Kp", "Ki", "Kd", "Velocity"]
        row_labels = ["FR", "FL", "RR", "RL"]

        self.pid_entries = []

        for col_idx in range(len(column_labels)):
            pid_label = Tk.Label(frame_motor_control, text = column_labels[col_idx], font = self.title_font)
            pid_label.grid(row = 0, column = 2 * col_idx + 1, columnspan = 2, sticky = 'nsew', padx = 3, pady = 3)

        for row_idx in range(1, len(row_labels) + 1):
            motor_label = Tk.Label(frame_motor_control, text = row_labels[row_idx - 1], font = self.title_font)
            motor_label.grid(row = row_idx, column = 0, sticky = 'nsew');

            for col_idx in range(len(column_labels)):
                entry = Tk.Entry(frame_motor_control, justify = Tk.CENTER, font = self.title_font)
                button_left = Tk.Button(frame_motor_control, text = "<", font = self.title_font, borderwidth = 3, relief = 'raised')
                button_right = Tk.Button(frame_motor_control, text = ">", font = self.title_font, borderwidth = 3, relief = 'raised')

                button_left.bind('<Button-1>', lambda event, idx = col_idx: self._buttonClickedHandler(event, "decrement {}".format(idx)))
                button_right.bind('<Button-1>', lambda event, idx = col_idx: self._buttonClickedHandler(event, "increment {}".format(idx)))

                button_left.bind('<ButtonRelease-1>', lambda event, idx = col_idx: self._buttonReleasedHandler(event, "decrement {}".format(idx)))
                button_right.bind('<ButtonRelease-1>', lambda event, idx = col_idx: self._buttonReleasedHandler(event, "increment {}".format(idx)))

                entry.grid(row = row_idx, column = 2 * col_idx + 1, columnspan = 2, sticky = 'nsew', padx = 20, pady = 3)
                #button_left.grid(row = 2, column = 2 * col_idx, sticky = 'nsew', padx = 20, pady = 3)
                #button_right.grid(row = 2, column = 2 * col_idx + 1, sticky = 'nsew', padx = 20, pady = 3)

                entry.insert(0, "0")
                self.pid_entries.append(entry)

        button = Tk.Button(frame_motor_control, text = "GO!", font = self.title_font, borderwidth = 3, relief = 'raised')
        button.bind('<Button-1>', lambda event, idx = col_idx: self._buttonClickedHandler(event, "go"))
        button.grid(row = len(row_labels) + 2, column = int(len(column_labels) / 2) + 1, columnspan = 4, sticky = 'nsew', padx = 20, pady = 5)

        for i in range(len(row_labels) + 2):
            frame_motor_control.rowconfigure(index = i, weight = 1)

        frame_motor_control.columnconfigure(index = 0, weight = 1, minsize = 50)
        for i in range(1, 2 * len(column_labels) + 1):
            frame_motor_control.columnconfigure(index = i, weight = 1)

        title_label = Tk.Label(frame_title, text = "RoboMecanum Control Panel", font = self.title_font)
        title_label.pack(fill = Tk.BOTH, side = Tk.TOP, expand = True)

        self.motor_fr_graph = FrameGraph(frame_graph_fr)
        self.motor_fl_graph = FrameGraph(frame_graph_fl)
        self.motor_rr_graph = FrameGraph(frame_graph_rr)
        self.motor_rl_graph = FrameGraph(frame_graph_rl)

    def _windowExitHandler(self):
        self._controller.windowExitHandler()

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

    def getPidParameterEntry(self, entry_idx):
        return int(self.pid_entries[entry_idx].get())

    def setPidParameterEntry(self, entry_idx, value):
        self.pid_entries[entry_idx].delete(0, 'end')
        self.pid_entries[entry_idx].insert(0, value)

    def getPidParameters(self):
        pid_params = []
        for entry in self.pid_entries:
            pid_params.append(int(entry.get()))
        return pid_params

    def getPidKp(self):
        return self.getPidParameterEntry(0)

    def getPidKi(self):
        return self.getPidParameterEntry(1)

    def getPidKd(self):
        return self.getPidParameterEntry(2)

    def getVelocity(self):
        return int(self.velocity_entry.get())

    def updateFrontRightMotorSpeed(self, value):
        self.motor_fr_graph.setSpeed(value)

    def updateFrontLeftMotorSpeed(self, value):
        self.motor_fl_graph.setSpeed(value)

    def updateRearRightMotorSpeed(self, value):
        self.motor_rr_graph.setSpeed(value)

    def updateRearLeftMotorSpeed(self, value):
        self.motor_rl_graph.setSpeed(value)

    def stop(self):
        plt.close("all")
        self.root.destroy()

    def start(self):
        self.root.deiconify()
        self.root.mainloop()


class Controller:

    CMD_SETPOINT = "setpoint={}\r"
    CMD_SETKP = "pidkp={}\r"
    CMD_SETKI = "pidki={}\r"
    CMD_SETKD = "pidkd={}\r"

    INFO_SPEED = "Speed"

    def __init__(self):
        self.PanelView = View(self)
        self.PanelModel = Model(self)

    def incrementPidParamEntryHandler(self, entry_idx):
        current_val = self.PanelView.getPidParameterEntry(entry_idx)
        updated_val = current_val + 1
        self.PanelView.setPidParameterEntry(entry_idx, updated_val)

    def decrementPidParamEntryHandler(self, entry_idx):
        current_val = self.PanelView.getPidParameterEntry(entry_idx)
        updated_val = current_val - 1
        self.PanelView.setPidParameterEntry(entry_idx, updated_val)

    def newUartDataHandler(self, line):
        line = line.rstrip()

        if line.startswith(self.INFO_SPEED):
            try:
                speed_value = int(line.split(" ")[-1])
                motor_desc = line.split(" ")[-2]

                if motor_desc.startswith("FR"):
                    self.PanelView.updateFrontRightMotorSpeed(speed_value)
                elif motor_desc.startswith("FL"):
                    self.PanelView.updateFrontLeftMotorSpeed(speed_value)
                elif motor_desc.startswith("RR"):
                    self.PanelView.updateRearRightMotorSpeed(speed_value)
                elif motor_desc.startswith("RL"):
                    self.PanelView.updateRearLeftMotorSpeed(speed_value)

            except ValueError:
                print("Cannot convert speed to value: {}".format(line))
        else:
            print("Controller got new uart line: >>{}<<".format(line))

    def setPidPoint(self):
        kp = self.PanelView.getPidKp()
        ki = self.PanelView.getPidKi()
        kd = self.PanelView.getPidKd()
        velocity = self.PanelView.getVelocity()
        self.PanelModel.sendStringOverUart(self.CMD_SETKP.format(kp))
        self.PanelModel.sendStringOverUart(self.CMD_SETKI.format(ki))
        self.PanelModel.sendStringOverUart(self.CMD_SETKD.format(kd))
        self.PanelModel.sendStringOverUart(self.CMD_SETPOINT.format(velocity))

    def windowExitHandler(self):
        self.PanelView.stop()
        self.PanelModel.stop()

    def run(self):
        self.PanelView.start()


if __name__ == '__main__':
    PanelController = Controller()
    PanelController.run()

