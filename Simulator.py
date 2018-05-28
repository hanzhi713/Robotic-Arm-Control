from math import *
import matplotlib

matplotlib.use("TkAgg")  # 可能Mac特有的要求，必须要很明显地说matplotlib用tkinter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tkinter import *
import numpy as np
from numpy.linalg import norm

half_pi = pi / 2
implementation = input("Enter tt, ax or t: \n")


class Arm:
    steps = 10
    default_position = np.array([90, 90, 90, 90, 90, 45], np.float64)

    @staticmethod
    def default_optimization(d1, d2, d3, p):
        return norm([d1, d2, d3], ord=1)

    @staticmethod
    def minimum_change(d1, d2, d3, p):
        return norm(90 - (np.rad2deg(np.array([d1, d2, d3])) - p[:3]), ord=2)

    def __init__(self, r1, r2, r3, opt=default_optimization, implementation="t"):
        """
          :param r1: Length of the first segment
          :param r2: Length of the second segment
          :param r3: Length of the third segment
          :param opt: The optimization function.
          """
        self.r1 = r1
        self.r2 = r2
        self.r3 = r3
        self.len = r1 + r2 + r3
        self.position = Arm.default_position.copy()

        # servo degree in radians
        self.rad_pos = np.array([half_pi, half_pi, half_pi, half_pi, half_pi, pi / 4], dtype=np.float64)
        self.opt = opt

        # Stands for Tom: Hanzhi Zhou's implementation by analytically solve the inequality using algebraic method
        if implementation == "t":
            self.solve_three = self.t_solve_angle

        # Stands for Alex: Yuhao Zhou's implementation by analytically solve the inequality using angles
        elif implementation == "ax":
            self.solve_three = self.ax_solve_angle

        # Stands for Tom Traversal: Hanzhi Zhou's original implementation by traversing all possible values of m
        elif implementation == "tt":
            self.solve_three = self.t_solve_angle
            self.get_m_range = self.get_m_range_traverse

    def get_radians(self, x, y, z):
        """
        :param x:
        :param y:
        :param z:
        :return: a list of radians
        """
        distance = norm((x, y, z), ord=2)
        if distance > self.len:
            raise Exception("Too far to reach!")

        d0 = atan2(y, x)
        # if not pi * 0.75 < d0 < -pi * 0.25:
        #     raise Exception("Out of range!")

        d1, d2, d3 = self.solve_three(norm((x, y), ord=2), z)
        return np.array((d0, d1, d2, d3), dtype=np.float64)

    def get_degrees(self, x, y, z):
        """
        Get servo rotation required to find (x, y, z)
        :param x:
        :param y:
        :param z:
        :return: A list of degrees
        """
        self.rads_to_degs(self.get_radians(x, y, z))

    @staticmethod
    def rads_to_degs(rads):
        return np.round(np.rad2deg(rads))

    def t_solve_angle(self, a, b):
        """
        :param a: x coordinate in the plane
        :param b: y coordinate in the plane
        :return: a list of degrees
        """

        s = inf
        rs = None
        rg = self.get_m_range(a, b)
        for m in rg:
            try:
                d1 = asin(m / self.r1)
                n = sqrt(self.r1 ** 2 - m ** 2)
                temp = (a - m) ** 2 + (b - n) ** 2
                d2 = half_pi - d1 - acos((self.r2 ** 2 + temp - self.r3 ** 2) / (2 * self.r2 * sqrt(temp))) - atan(
                    (b - n) / (a - m))
                if not -half_pi < d2 < half_pi:
                    continue
                d3 = pi - acos((self.r2 ** 2 + self.r3 ** 2 - temp) / (2 * self.r2 * self.r3))
                if not -half_pi < d3 < half_pi:
                    continue
                opt_val = self.opt(d1, d2, d3, self.position)
                if opt_val < s:
                    s = opt_val
                    rs = d1, d2, d3
            except:
                continue
        return rs

    def get_m_range_traverse(self, a, b):
        rg = []
        c1 = norm((self.r2, self.r3), ord=2)
        c2 = (self.r2 + self.r3) ** 2
        for i in range(-self.r1 * Arm.steps, self.r1 * Arm.steps):
            m = i / Arm.steps
            v = (a - m) ** 2 + (b - sqrt(self.r1 ** 2 - m ** 2)) ** 2
            if c1 < v < c2:
                rg.append(m)
        return rg

    def get_m_range(self, a, b):
        l1 = self.r1
        l2 = self.r2
        l3 = self.r3

        A = l2 ** 2 + l3 ** 2 - l1 ** 2 - a ** 2 - b ** 2

        temp = a ** 2 + b ** 2
        d1 = 4 * temp * l1 ** 2 - A ** 2

        if d1 > 0:
            m11 = (-A * a - b * sqrt(d1)) / (2 * temp)
            m12 = (-A * a + b * sqrt(d1)) / (2 * temp)
            print(m11, m12)

        B = -A - 2 * l2 * l3
        d2 = 4 * temp * l1 ** 2 - B ** 2

        u = B / (2*a)

        if d2 > 0:
            m21 = (B * a - b * sqrt(d2)) / (2 * temp)
            m22 = (B * a + b * sqrt(d2)) / (2 * temp)

            print(m21, m22)

        print(u - l1)

        if b > 0:
            # find the intersection between [-oo, m11], [m21, m22] and [m12, +oo]
            if d1 > 0 and d2 > 0:
                if m12 < m21 or m11 > m22:
                    print(1)
                    return self.process_m_range([m21, m22])

                elif m11 < m21 < m12 < m22:
                    print(2)
                    return self.process_m_range([m12, m22])

                elif m21 < m11 and m12 < m22:
                    print(3)
                    return self.process_m_range([m21, m11, m12, m22])

                elif m21 < m11 < m22 < m12:
                    print(4)
                    return self.process_m_range([m21, m11])

                elif m11 < m21 and m12 > m22:
                    print(5)
                    return []

            elif d1 > 0 and d2 < 0:
                print(6)
                return self.process_m_range([-self.r1, m11, m12, self.r1])

            elif d1 < 0 and d2 > 0:
                print(7)
                return self.process_m_range([m21, m22])

            else:
                print(8)
                return []
        else:
            print(0)
            m1 = u
            m2 = -A / (2 * a)
            print(m1, m2)
            return self.process_m_range([m1, m2])

    def process_m_range(self, m_range):
        if len(m_range) == 2:
            if m_range[0] < -self.r1: m_range[0] = -self.r1
            if m_range[1] > self.r1: m_range[1] = self.r1
            return np.arange(m_range[0] + 1 / Arm.steps, m_range[1], 1 / Arm.steps)
        elif len(m_range) == 4:
            return np.concatenate((np.arange(m_range[0] + 1 / Arm.steps, m_range[1], 1 / Arm.steps),
                                   np.arange(m_range[2] + 1 / Arm.steps, m_range[3], 1 / Arm.steps)))
        else:
            raise Exception('!!!')

    # For Alex's implementation
    def ax_solve_angle(self, a, b):
        theta_range = self.get_angle_range(a, b)
        s = inf
        rs = None
        for k in theta_range:
            try:
                d1 = np.deg2rad(k)
                m, n = self.r1 * sin(d1), self.r1 * cos(d1)
                temp = (norm(((a - m), (b - n)), ord=2)) ** 2
                d2 = half_pi - d1 - acos((self.r2 ** 2 + temp - self.r3 ** 2) / (2 * self.r2 * sqrt(temp))) - atan(
                    (b - n) / (a - m))
                if not -half_pi < d2 < half_pi:
                    continue
                d3 = pi - acos((self.r2 ** 2 + self.r3 ** 2 - temp) / (2 * self.r2 * self.r3))
                if not -half_pi < d3 < half_pi:
                    continue
                opt_val = self.opt(d1, d2, d3, self.position)
                if opt_val < s:
                    s = opt_val
                    rs = d1, d2, d3
            except:
                continue
        return rs

    # For Alex's implementation
    def get_angle_range(self, a, b):
        theta_range = []
        k = norm((a, b), ord=2)  # distance
        phi = atan(b / a)

        lim_min = norm((self.r2, self.r3), ord=2) ** 2
        lim_max = (self.r2 + self.r3) ** 2
        m_min = (k ** 2 + self.r1 ** 2 - lim_max) / (2 * k * self.r1)
        m_max = (k ** 2 + self.r1 ** 2 - lim_min) / (2 * k * self.r1)

        if m_min < -1:  # Will this be a case?
            m_min = -1
        angle_1 = asin(m_min)

        angle_3 = 0
        angle_4 = 0
        if m_max > 1:
            print(m_max)
            if pi - angle_1 - phi > pi / 2:  # Will this be a case?
                angle_2 = pi / 2
            else:
                angle_2 = pi - angle_1
        else:
            angle_2 = asin(m_max)

        if pi / 2 + phi > pi - angle_1:
            angle_3 = pi - angle_2
            angle_4 = pi - angle_1
        elif pi - angle_2 <= pi / 2 + phi <= pi - angle_1:
            angle_3 = pi - angle_2
            angle_4 = pi / 2 + phi

        angle_1 = angle_1 - phi
        angle_2 = angle_2 - phi

        c = np.rad2deg(np.array([angle_1, angle_2]))
        if angle_3 != 0 and angle_4 != 0:
            c = np.concatenate((c, np.rad2deg(np.array([angle_3, angle_4]))), axis=0)
            for i in range(ceil(10 * c[0]), floor(10 * c[1] + 1)):
                theta_range.append(i / 10)
            for i in range(ceil(10 * c[2]), floor(10 * c[3] + 1)):
                theta_range.append(i / 10)
        else:
            for i in range(ceil(10 * c[0]), floor(10 * c[1] + 1)):
                theta_range.append(i / 10)

        return theta_range

    # convert degrees to servo angles
    @staticmethod
    def cov_degs(degs):
        degs = degs.copy()
        degs[0] += 45
        degs[1] = 90 - degs[1]
        degs[2] = 90 - degs[2]
        degs[3] = 90 - degs[3]
        return degs

    @staticmethod
    def cov_rads(rads):
        rads = rads.copy()
        rads[0] += pi / 4
        rads[1] = half_pi - rads[1]
        rads[2] = half_pi - rads[2]
        rads[2] = half_pi - rads[3]

    # Update the joint angles so that the arm can reach (x, y, z)
    def goto(self, x, y, z):
        self.rad_pos = self.get_radians(x, y, z)
        self.position = np.concatenate((self.cov_degs(self.rads_to_degs(self.rad_pos)), self.position[4:]), axis=0)

    # get coordinates of each joint in three dimensional space
    def get_coordinates(self):
        d0 = self.rad_pos[0]
        d1 = pi / 2 - self.rad_pos[1]
        x1, y1 = cos(d1) * self.r1 * cos(d0), cos(d1) * self.r1 * sin(d0)
        z1 = sin(d1) * self.r1
        d2 = d1 - self.rad_pos[2]
        x2, y2 = cos(d2) * self.r2 * cos(d0) + x1, cos(d2) * self.r2 * sin(d0) + y1
        z2 = z1 + sin(d2) * self.r2
        r3 = self.r3
        d3 = d2 - self.rad_pos[3]
        x3, y3 = cos(d3) * r3 * cos(d0) + x2, cos(d3) * r3 * sin(d0) + y2
        z3 = z2 + sin(d3) * r3
        return [0, x1, x2, x3], [0, y1, y2, y3], [0, z1, z2, z3]

    def write(self, sr):
        sr.write(self.position)


write_serial = False
if write_serial:
    from Protocol import ServoProtocol

    sr = ServoProtocol('COM3')


def callback(e):
    global arm, x, y, z, sr, write_serial
    arm.goto(x, y, z)
    xs, ys, zs = arm.get_coordinates()
    ax.clear()
    ax.set_xlabel('X / mm')
    ax.set_xlim(-20, 200)
    ax.set_ylabel('Y / mm')
    ax.set_ylim(-20, 200)
    ax.set_zlabel('Z / mm')
    ax.set_zlim(0, 200)
    ax.plot(xs, ys, zs, linewidth=2.5, marker='*', markersize=8, markerfacecolor='y')
    ax.scatter(x, y, z, c='r', s=100, marker='o')

    if write_serial:
        arm.write(sr)


def update(n, t):
    global x, y, z
    if n == 1:
        x = int(t)
    elif n == 2:
        y = int(t)
    else:
        z = int(t)
    callback(1)


arm = Arm(93, 87, 139, Arm.minimum_change, implementation)

fig = plt.figure()
ax = Axes3D(fig)

plt.ion()
x = 150
y = 150
z = 150

root = Tk()
root.title('Robotic Arm Control Simulation')

sx = Scale(root, from_=0, to_=300, orient=HORIZONTAL, length=600, command=lambda t: update(1, t))
sy = Scale(root, from_=0, to_=300, orient=HORIZONTAL, length=600, command=lambda t: update(2, t))
sz = Scale(root, from_=0, to_=300, orient=HORIZONTAL, length=600, command=lambda t: update(3, t))

sx.bind('<ButtonRelease-1>', callback)
sy.bind('<ButtonRelease-1>', callback)
sz.bind('<ButtonRelease-1>', callback)

sx.set(150)
sy.set(150)
sz.set(150)

sx.pack()
sy.pack()
sz.pack()

plt.show()
root.mainloop()
