from math import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tkinter import *
import numpy as np
from numpy.linalg import norm

half_pi = pi / 2


class Arm:
    steps = 100
    default_position = np.array([90, 90, 90, 90, 90, 45], np.float64)

    @staticmethod
    def default_optimization(d1, d2, d3, p):
        return norm([d1, d2, d3], ord=1)

    @staticmethod
    def minimum_change(d1, d2, d3, p):
        return norm(90 - (np.rad2deg(np.array([d1, d2, d3])) - p[:3]), ord=2)

    def __init__(self, l1, l2, l3, opt=default_optimization):
        """
        :param l1: Length of the first segment
        :param l2: Length of the second segment
        :param l3: Length of the third segment
        :param opt: The optimization function.
        """
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.len = l1 + l2 + l3
        self.position = Arm.default_position.copy()

        # servo degree in radians
        self.rad_pos = np.array([half_pi, half_pi, half_pi, half_pi, half_pi, pi / 4], dtype=np.float64)
        self.opt = opt

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

    def solve_three(self, a, b):
        """
        :param a: x coordinate in the plane
        :param b: y coordinate in the plane
        :return: a list of degrees
        """
        rg = self.get_m_range(a, b)
        s = inf
        rs = None
        for m in rg:
            d1 = asin(m / self.l1)
            n = sqrt(self.l1 ** 2 - m ** 2)
            temp = (a - m) ** 2 + (b - n) ** 2
            d2 = half_pi - d1 - acos((self.l2 ** 2 + temp - self.l3 ** 2) / (2 * self.l2 * sqrt(temp))) - atan(
                (b - n) / (a - m))
            if not -half_pi < d2 < half_pi:
                continue
            d3 = pi - acos((self.l2 ** 2 + self.l3 ** 2 - temp) / (2 * self.l2 * self.l3))
            if not -half_pi < d3 < half_pi:
                continue
            opt_val = self.opt(d1, d2, d3, self.position)
            if opt_val < s:
                s = opt_val
                rs = d1, d2, d3
        return rs

    def get_m_range(self, a, b):
        rg = []
        c1 = norm((self.l2, self.l3), ord=2)
        c2 = (self.l2 + self.l3) ** 2
        for i in range(-self.l1 * Arm.steps, self.l1 * Arm.steps):
            m = i / Arm.steps
            v = (a - m) ** 2 + (b - sqrt(self.l1 ** 2 - m ** 2)) ** 2
            if c1 < v < c2:
                rg.append(m)
        return rg

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
        self.position = np.concatenate((self.cov_degs(self.rads_to_degs(self.rad_pos)), self.position[4:]))

    # get coordinates of each joint in three dimensional space
    def get_coordinates(self):
        d0 = self.rad_pos[0]
        d1 = pi / 2 - self.rad_pos[1]
        x1, y1 = cos(d1) * self.l1 * cos(d0), cos(d1) * self.l1 * sin(d0)
        z1 = sin(d1) * self.l1
        d2 = d1 - self.rad_pos[2]
        x2, y2 = cos(d2) * self.l2 * cos(d0) + x1, cos(d2) * self.l2 * sin(d0) + y1
        z2 = z1 + sin(d2) * self.l2
        l3 = self.l3
        d3 = d2 - self.rad_pos[3]
        x3, y3 = cos(d3) * l3 * cos(d0) + x2, cos(d3) * l3 * sin(d0) + y2
        z3 = z2 + sin(d3) * l3
        return [0, x1, x2, x3], [0, y1, y2, y3], [0, z1, z2, z3]

    def write(self, sr):
        sr.write(self.position)


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


write_serial = False
if write_serial:
    from Protocol import ServoProtocol

    sr = ServoProtocol('COM3')

arm = Arm(93, 87, 139, Arm.minimum_change)

fig = plt.figure()
ax = Axes3D(fig)

plt.ion()
x = 150
y = 150
z = 150

root = Tk()
root.title('Robotic Arm Control Simulation')

sx = Scale(root, from_=-100, to_=300, orient=HORIZONTAL, length=600, command=lambda t: update(1, t))
sy = Scale(root, from_=-100, to_=300, orient=HORIZONTAL, length=600, command=lambda t: update(2, t))
sz = Scale(root, from_=-100, to_=300, orient=HORIZONTAL, length=600, command=lambda t: update(3, t))
# sx.bind('<ButtonRelease-1>', callback)
# sy.bind('<ButtonRelease-1>', callback)
# sz.bind('<ButtonRelease-1>', callback)
sx.set(150)
sy.set(150)
sz.set(150)
sx.pack()
sy.pack()
sz.pack()
plt.show()
root.mainloop()
