from math import *
import matplotlib.axes

matplotlib.use("TkAgg")  # Mac requires matplotlib to explicitly use tkinter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tkinter import *
import numpy as np
from numpy.linalg import norm
import time

half_pi = pi / 2
impl = "ax"  # input("Enter tt, ax or t: \n")


class Arm:
    steps = 10
    step_length = 1 / steps
    default_position = np.array([90, 90, 90, 90, 90, 79], np.int32)

    @staticmethod
    def default_optimization(d1, d2, d3, p):
        return norm([d1, d2, d3], ord=1)

    @staticmethod
    def minimum_change(d1, d2, d3, p):
        return norm(90 - (np.rad2deg(np.array([d1, d2, d3])) - p[:3]), ord=2)

    def __init__(self, r1, r2, r3, x=0, y=150, z=150, opt=default_optimization, implementation="ax"):
        """
          :param r1: Length of the first segment
          :param r2: Length of the second segment
          :param r3: Length of the third segment
          :param opt: The optimization function.
          """
        self.r1 = r1
        self.r2 = r2
        self.r3 = r3
        self.x, self.y, self.z = x, y, z
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
        
        else:
            raise ValueError('implementation must be one of "t", "ax" and "tt"')

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
        return np.round(np.rad2deg(rads), 0)

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

        m11 = 0
        m12 = 0
        m21 = 0
        m22 = 0

        if d1 > 0:
            m11 = int((-A * a - b * sqrt(d1)) / (2 * temp) * Arm.steps)
            m12 = int((-A * a + b * sqrt(d1)) / (2 * temp) * Arm.steps)

        B = -A - 2 * l2 * l3
        d2 = 4 * temp * l1 ** 2 - B ** 2

        u = int(B / (2 * a) * Arm.steps)

        if d2 > 0:
            m21 = int((B * a - b * sqrt(d2)) / (2 * temp) * Arm.steps)
            m22 = int((B * a + b * sqrt(d2)) / (2 * temp) * Arm.steps)

        if b != 0:
            first_solutions = set(range(-l1 * Arm.steps, m11)).union(set(range(m12, l1 * Arm.steps)))
            second_solutions = set(range(m21, m22)).union(set(range(u, l1 * Arm.steps)))
            return np.array(list(first_solutions.intersection(second_solutions))) / Arm.steps
        else:
            print(0)
            m1 = u
            m2 = -A / (2 * a)
            return np.arange(m1 + Arm.step_length, m2, Arm.step_length)

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
            for i in range(ceil(Arm.steps * c[0]), floor(Arm.steps * c[1] + 1)):
                theta_range.append(i / Arm.steps)
            for i in range(ceil(Arm.steps * c[2]), floor(Arm.steps * c[3] + 1)):
                theta_range.append(i / Arm.steps)
        else:
            for i in range(ceil(Arm.steps * c[0]), floor(Arm.steps * c[1] + 1)):
                theta_range.append(i / Arm.steps)

        return theta_range

    # convert degrees to servo angles
    @staticmethod
    def cov_degs(degs):
        degs = degs.copy()
        # degs[0] += 45
        degs[1] = 90 - degs[1]
        degs[2] = 90 - degs[2]
        degs[3] = 90 - degs[3]
        return degs

    @staticmethod
    def cov_rads(rads):
        rads = rads.copy()
        # rads[0] += pi / 4
        rads[1] = half_pi - rads[1]
        rads[2] = half_pi - rads[2]
        rads[2] = half_pi - rads[3]

    # Update the joint angles so that the arm can reach (x, y, z)
    def goto(self, x=None, y=None, z=None):
        if x is None: x = self.x
        if y is None: y = self.y
        if z is None: z = self.z

        self.rad_pos = self.get_radians(x, y, z)
        self.position = np.concatenate((self.cov_degs(self.rads_to_degs(self.rad_pos)), self.position[4:]),
                                       axis=0).astype(np.int32)
        self.x, self.y, self.z = x, y, z

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

    def set_claws_rotation(self, rot):
        self.position[4] = rot

    def open_claws(self):
        self.position[5] = 0

    def close_claws(self):
        self.position[5] = 79

    def write(self, sr):
        sr.write(self.position)

    def pickup(self, sr, target_x, target_y, target_z, claw_rot, smooth=True, steps=40, high=80):
        self.open_claws()
        self.set_claws_rotation(claw_rot)
        self.goto_and_write(sr, target_x, target_y, target_z + high, smooth=smooth, steps=steps)
        self.goto_and_write(sr, target_x, target_y, target_z, smooth=smooth, steps=8)
        time.sleep(0.75)
        self.close_claws()
        self.goto_and_write(sr, target_x, target_y, target_z + high, smooth=smooth, steps=8)

    def goto_and_write(self, sr, target_x, target_y, target_z, smooth=True, steps=40):
        if smooth:
            x_step, y_step, z_step = (target_x - self.x) / steps, (target_y - self.y) / steps, (
                    target_z - self.z) / steps
            for x, y, z in zip(np.arange(self.x, target_x, x_step),
                               np.arange(self.y, target_y, y_step),
                               np.arange(self.z, target_z, z_step)):
                time.sleep(0.05)
                self.goto(x, y, z)
                self.write(sr)
        else:
            self.goto(target_x, target_y, target_z)
            self.write(sr)

def vec_rot(vec, axis, theta):
    return vec * cos(theta) + np.cross(axis, vec) * sin(theta) + axis * np.dot(axis, vec) * (1 - cos(theta))


class Simulator:

    def __init__(self, arm: Arm, ax: Axes3D):
        self.arm = arm
        self.ax = ax
        self.update()

    def update(self, xmin=-100, xmax=100, ymin=0, ymax=200, zmin=-100, zmax=100):
        xs, ys, zs = self.arm.get_coordinates()
        points = np.array(list(zip(xs, ys, zs))[1:])

        last_line = points[2] - points[1]
        last_line /= np.linalg.norm(last_line) # unit vector
        normal = np.cross(points[0] - points[1], points[0] - points[2])

        rot_axis = np.cross(normal, last_line)
        rot_axis /= np.linalg.norm(rot_axis)        
        
        # open and close rotation
        _rot1 = np.deg2rad(self.arm.position[5]) / 2
        temp1 = vec_rot(last_line * 20, rot_axis, _rot1)
        temp2 = vec_rot(last_line * 20, rot_axis, - _rot1)

        # fifth servo rotation
        rot = np.deg2rad(self.arm.position[4])
        claw_vec_rot1 = vec_rot(temp1, last_line, rot)
        claw_vec_rot2 = vec_rot(temp2, last_line, rot)

        claw_point1 = points[2] + claw_vec_rot1
        claw_point2 = points[2] + claw_vec_rot2

        self.ax.clear()
        self.ax.set_xlabel('X / mm')
        self.ax.set_xlim(xmin, xmax)
        self.ax.set_ylabel('Y / mm')
        self.ax.set_ylim(ymin, ymax)
        self.ax.set_zlabel('Z / mm')
        self.ax.set_zlim(zmin, zmax)
        self.ax.plot(xs, ys, zs, linewidth=2.5, marker='*', markersize=8, markerfacecolor='y')
        self.ax.scatter(xs[-1], ys[-1], zs[-1], c='r', s=100, marker='o')

        claw_points = np.array([points[2], claw_point1]).transpose()
        self.ax.plot(claw_points[0], claw_points[1], claw_points[2], linewidth=2.5, marker='*', markersize=8, markerfacecolor='y')
        claw_points = np.array([points[2], claw_point2]).transpose()
        self.ax.plot(claw_points[0], claw_points[1], claw_points[2], linewidth=2.5, marker='*', markersize=8, markerfacecolor='y')


if __name__ == "__main__":
    write_serial = False
    if write_serial:
        from protocol import ServoProtocol

        sr = ServoProtocol('COM3')


    def update(n, t):
        global claw_status, arm, simulator
        if n == 1:
            arm.goto(x=int(t))
        elif n == 2:
            arm.goto(y=int(t))
        elif n == 3:
            arm.goto(z=int(t))
        elif n == 4:
            arm.set_claws_rotation(int(t))
        elif n == 5:
            claw_status = not claw_status
            if claw_status:
                arm.open_claws()
            else:
                arm.close_claws()
        simulator.update()

        if write_serial:
            arm.write(sr)


    fig = plt.figure()
    ax = Axes3D(fig)
    arm = Arm(93, 87, 110, opt=Arm.minimum_change, implementation=impl)
    simulator = Simulator(arm, ax)

    plt.ion()
    claw_status = False

    root = Tk()
    root.title('Robotic Arm Control Simulation')

    sx = Scale(root, from_=-200, to_=200, orient=HORIZONTAL, length=600, command=lambda t: update(1, t))
    sy = Scale(root, from_=0, to_=300, orient=HORIZONTAL, length=600, command=lambda t: update(2, t))
    sz = Scale(root, from_=-75, to_=200, orient=HORIZONTAL, length=600, command=lambda t: update(3, t))
    claw_s = Scale(root, from_=0, to_=180, orient=HORIZONTAL, length=600, command=lambda t: update(4, t))
    Button(root, text="Change Claw Status", command=lambda: update(5, None)).pack()

    sx.set(0)
    sy.set(150)
    sz.set(100)
    claw_s.set(90)

    sx.pack()
    sy.pack()
    sz.pack()
    claw_s.pack()

    plt.show()
    root.mainloop()
