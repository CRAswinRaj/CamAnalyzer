import matplotlib
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib import animation
from tkinter import *
from tkinter import ttk
import math

t_list = np.linspace(0, 2 * np.pi, 1000)


def rot_matrix(theta):  # Rotational Matrix
    return [[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]]


class Cam:
    def __init__(self, rot_type, motion_type, Rb, Rr, e, sorted_data):
        self.Y = []
        self.rot_type = rot_type
        self.motion_type = motion_type
        self.Rb = Rb
        self.Rr = Rr
        self.e = e
        self.d = np.sqrt((self.Rb + self.Rr) ** 2 - self.e ** 2)
        self.sorted_dwell_data = sorted_data
        self.camCoords = self.camCurve()

    # To find the coordinates of Cam
    def camCurve(self):
        XCoord = []
        YCoord = []
        a = np.array([[0], [2 * np.pi], [2 * np.pi]])

        dwell_data = np.concatenate(
            (self.sorted_dwell_data[:, -1].reshape(-1, 1) - a, self.sorted_dwell_data,
             self.sorted_dwell_data[:, 0].reshape(-1, 1) + a), axis=1)

        n = 1
        for t in t_list:
            if t > dwell_data[2, n]:
                n += 1
            if t <= dwell_data[1, n]:
                y, dy, d2y = self.y_dy(t, dwell_data[2, n - 1], dwell_data[1, n], dwell_data[0, n - 1],
                                       dwell_data[0, n])
            else:
                y, dy, d2y = dwell_data[0, n], 0, 0

            phi = np.arctan2((dy - self.rot_type * self.e), (self.d + y))

            XC = (self.e + self.Rr * np.sin(self.rot_type * phi)) * np.cos(t) + (
                    self.d + y - self.Rr * np.cos(phi)) * np.sin(self.rot_type * t)
            YC = -(self.e + self.Rr * np.sin(self.rot_type * phi)) * np.sin(self.rot_type * t) + (
                    self.d + y - self.Rr * np.cos(phi)) * np.cos(t)
            if self.check_undercutting(y, dy, d2y):
                return -1
            XCoord.append(XC)
            YCoord.append(YC)
            self.Y.append(y)

        XCoord.append(XCoord[0])
        YCoord.append(YCoord[0])
        return [XCoord, YCoord]

    # Return follower displacement (y) and its derivative (dy/dtheta)
    def y_dy(self, t, t1, t2, L1, L2):
        if self.motion_type == "3-4-5":
            y = L1 + (L2 - L1) * (10 * ((t - t1) / (t2 - t1)) ** 3 - 15 * ((t - t1) / (t2 - t1)) ** 4 + 6 * (
                    (t - t1) / (t2 - t1)) ** 5)
            dy = (L2 - L1) / (t2 - t1) * (30 * ((t - t1) / (t2 - t1)) ** 2 - 60 * ((t - t1) / (t2 - t1)) ** 3 + 30 * (
                    (t - t1) / (t2 - t1)) ** 4)
            d2y = (L2 - L1) / (t2 - t1) ** 2 * (
                    60 * ((t - t1) / (t2 - t1)) - 180 * ((t - t1) / (t2 - t1)) ** 2 + 120 * (
                        (t - t1) / (t2 - t1)) ** 3)
        elif self.motion_type == "Cycloidal":
            y = L1 + (L2 - L1) * ((t - t1) / (t2 - t1) - 1 / (2 * np.pi) * np.sin(2 * np.pi * (t - t1) / (t2 - t1)))
            dy = (L2 - L1) / (t2 - t1) * (1 - np.cos(2 * np.pi * (t - t1) / (t2 - t1)))
            d2y = 2 * np.pi * (L2 - L1) / (t2 - t1) ** 2 * np.sin(2 * np.pi * (t - t1) / (t2 - t1))
        else:
            return None

        return y, dy, d2y

    # To find coordinates of follower (Circle and a line)
    def follower_func(self, y):
        d = np.sqrt((self.Rb + self.Rr) ** 2 - self.e ** 2)
        XCoord = [self.e]
        YCoord = [self.d + y + self.Rr * 2.5]
        for t in np.linspace(0, 2 * np.pi, 50):
            XCoord.append(self.Rr * np.sin(t) + self.e)
            YCoord.append(self.Rr * np.cos(t) + d + y)
        return XCoord, YCoord

    def check_undercutting(self, y, dy, d2y):
        rc = ((y + self.d) ** 2 + (dy - self.rot_type * self.e) ** 2) ** 1.5 / (
                d2y * (y + self.d) - (y + self.d) ** 2 - (dy - self.rot_type * self.e) * (
                    2 * dy - self.rot_type * self.e)) - self.Rr
        if -self.Rr < rc < 0:
            return True


class Interface:
    def __init__(self, window):
        # Interface initialisations
        self.dwellno = 0
        self.window = window
        self.fig = Figure(figsize=(7, 7))
        self.a = self.fig.add_subplot(111)
        self.a.set_title("Cam Profile", fontsize=16)
        self.a.set_xlim([-15, 15])
        self.a.set_ylim([-15, 15])
        self.cam_curve, = self.a.plot([], [], color='b')
        self.follower_curve, = self.a.plot([], [], color='r')
        self.cam_centre = self.a.scatter([], [], color='b')
        canvas = FigureCanvasTkAgg(self.fig, master=self.window)
        canvas.get_tk_widget().grid(column=0, row=0, rowspan=28)
        self.ani = None

        # Cam Variables
        self.cam = None
        self.camCoords = None
        self.rtype = None
        self.rot_type = StringVar()
        self.motion_type = StringVar()
        self.Rb = DoubleVar()
        self.Rr = DoubleVar()
        self.e = DoubleVar()
        self.amp = []
        self.start_angle = []
        self.end_angle = []
        self.sorted_dwell_data = None

        # Sense of rotation
        Label(window, text="Rotation direction").grid(row=0, column=1)
        ttk.Combobox(window, width=17, textvariable=self.rot_type, values=("Clockwise", "Anticlockwise")).grid(row=0,
                                                                                                               column=2)

        # Type of follower motion
        Label(window, text="Type of follower motion").grid(row=1, column=1)
        ttk.Combobox(window, width=17, textvariable=self.motion_type, values=("Cycloidal", "3-4-5")).grid(row=1,
                                                                                                          column=2)

        # Base circle radius
        Label(window, text="Base circle radius").grid(row=2, column=1)
        Entry(window, textvariable=self.Rb).grid(row=2, column=2)

        # Roller radius
        Label(window, text="Roller radius").grid(row=3, column=1)
        Entry(window, textvariable=self.Rr).grid(row=3, column=2)

        # Offset
        Label(window, text="Offset").grid(row=4, column=1)
        Entry(window, textvariable=self.e).grid(row=4, column=2)

        self.B_remove_dwell = Button(window, text="Remove Dwell", width=12, command=self.remove_dwell_input)
        self.B_add_dwell = Button(window, text="Add Dwell", width=12, command=self.add_dwell_input)
        self.B_analyse = Button(window, text="Analyze", width=12, command=self.generate)
        self.error = Label(self.window, fg="red")
        self.error.grid(row=27, column=1, columnspan=2)
        for i in range(2):
            self.add_dwell_input()

    def add_dwell_input(self):  # Callback for add dwell button -> add new entries for dwell related data
        if self.dwellno == 4:
            self.set_error("Maximum number of dwells reached")
            return
        self.set_error(None)

        self.dwellno += 1
        self.amp.append(DoubleVar())
        self.start_angle.append(DoubleVar())
        self.end_angle.append(DoubleVar())

        Label(self.window, text=f"Dwell {self.dwellno}", font=('Helventica', 11, 'bold')).grid(row=4 * self.dwellno + 1,
                                                                                               column=1, columnspan=2)
        Label(self.window, text="Amplitude").grid(row=4 * self.dwellno + 2, column=1)
        Entry(self.window, textvariable=self.amp[self.dwellno - 1]).grid(row=4 * self.dwellno + 2, column=2)
        Label(self.window, text="Start angle(°)").grid(row=4 * self.dwellno + 3, column=1)
        Label(self.window, text="End angle(°)").grid(row=4 * self.dwellno + 3, column=2)
        Entry(self.window, width=10, textvariable=self.start_angle[self.dwellno - 1]).grid(row=4 * self.dwellno + 4,
                                                                                           column=1)
        Entry(self.window, width=10, textvariable=self.end_angle[self.dwellno - 1]).grid(row=4 * self.dwellno + 4,
                                                                                         column=2)
        self.B_remove_dwell.grid(row=4 * self.dwellno + 6, column=2)
        self.B_add_dwell.grid(row=4 * self.dwellno + 7, column=2)
        self.B_analyse.grid(row=4 * self.dwellno + 8, column=2)

    def remove_dwell_input(self):  # Callback for remove dwell button -> remove entries for previous dwell related data
        if self.dwellno == 2:
            self.set_error("At least 2 dwell is required")
            return
        self.set_error(None)

        for label in self.window.grid_slaves():
            if 4 * self.dwellno < int(label.grid_info()["row"]) < 4 * self.dwellno + 5:
                label.grid_forget()
        self.dwellno -= 1
        self.amp.pop()
        self.start_angle.pop()
        self.end_angle.pop()

        self.B_remove_dwell.grid(row=4 * self.dwellno + 6, column=2)
        self.B_add_dwell.grid(row=4 * self.dwellno + 7, column=2)
        self.B_analyse.grid(row=4 * self.dwellno + 8, column=2)
        return

    def set_error(self, error):  # Show 'error' at the bottom
        if error is None:
            self.error.config(text="")
        else:
            self.error.config(text="Error: " + error)

    def check_error(self):  # Identify the possible errors related to user imputs
        try:
            if self.rot_type.get() != "Clockwise" and self.rot_type.get() != "Anticlockwise":
                return "Unknown rotational direction"
            if self.motion_type.get() != "3-4-5" and self.motion_type.get() != "Cycloidal":
                return "Unknown follower motion type"
            if self.Rb.get() <= 0:
                return "Base circle radius should be positive"
            if self.Rr.get() <= 0:
                return "Roller circle radius should be positive"
            if self.e.get() >= self.Rb.get() + self.Rr.get():
                return "Offset should be less than \n prime circle radius"
            for i in range(self.dwellno):
                if self.amp[i].get() < 0:
                    return "Dwell Amplitude should be positive"
                if not 0 <= self.start_angle[i].get() <= 360 or not 0 <= self.end_angle[i].get() <= 360:
                    return "Angles should lie between 0 and 360"
            for i in range(self.dwellno):
                if (i != 0 and self.sorted_dwell_data[1, i] <= self.sorted_dwell_data[2, i - 1]) or \
                        self.sorted_dwell_data[1, i] > self.sorted_dwell_data[2, i]:
                    return "Dwells are overlapping"
            if self.sorted_dwell_data[1, 0] == 0 and self.sorted_dwell_data[2, -1] == 2 * np.pi:
                return "Dwells are overlapping"

        except TclError:
            return "Input Datatype Error"

    # arrange the dwell data as 2d matrix (amp, start angle and end angle) in the order of start angles
    def sort_dwell_data(self):
        try:
            data = np.array(
                [[i.get() for i in self.amp], [i.get() * np.pi / 180 for i in self.start_angle],
                 [i.get() * np.pi / 180 for i in self.end_angle]])
        except TclError:
            return

        self.sorted_dwell_data = data[:, data[1, :].argsort()]

        if self.rot_type.get() == "Clockwise":
            self.rtype = -1
        elif self.rot_type.get() == "Anticlockwise":
            self.rtype = 1

    # Generate Cam Profile
    def generate(self):
        if self.ani is not None:
            self.ani.event_source.stop()

        self.sort_dwell_data()
        error = self.check_error()
        self.set_error(error)
        if error is not None:
            return

        self.cam = Cam(self.rtype, self.motion_type.get(), self.Rb.get(), self.Rr.get(), self.e.get(),
                       self.sorted_dwell_data)
        if self.cam.camCoords == -1:
            self.set_error("Cam with given data \n lead to undercutting")
            return

        lim = self.Rb.get() + self.sorted_dwell_data[0].max() + 3 * self.Rr.get()
        self.a.set_xlim([-lim, lim])
        self.a.set_ylim([-lim, lim])

        self.ani = animation.FuncAnimation(self.fig, self.animate, init_func=self.curve_init, interval=15, frames=200,
                                           blit=True)

    #   Curve initialisation in animation
    def curve_init(self):
        self.cam_curve.set_data([], [])
        self.follower_curve.set_data([], [])
        return [self.cam_curve, self.follower_curve]

    #   Update animation
    def animate(self, i):
        [x, y] = np.dot(rot_matrix(self.rtype * i * 2 * np.pi / 200), self.cam.camCoords)
        x1, y1 = self.cam.follower_func(self.cam.Y[i * 5])
        self.cam_curve.set_data(x, y)
        self.follower_curve.set_data(x1, y1)
        return [self.cam_curve, self.follower_curve, self.a.scatter(0, 0, color='b'),
                self.a.scatter(self.cam.e, self.cam.d + self.cam.Y[i * 5], color='r')]


matplotlib.use('TkAgg')
Window = Tk()
Window.title("Cam Analyzer")
start = Interface(Window)
Window.mainloop()
