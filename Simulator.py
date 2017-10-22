from math import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tkinter import *
from Protocol import ServoProtocol


class Arm:
    steps = 100
    default_position = [90, 90, 90, 90, 90, 45]
    default_optimization = lambda d1, d2, d3, p: abs(d1) + abs(d2) + abs(d3)
    minimum_change = lambda d1, d2, d3, p: (90 - degrees(d1) - p[1]) ** 2 + (90 - degrees(d2) - p[2]) ** 2 + (90 - degrees(d3) - p[3]) ** 2

    def __init__(self, l1, l2, l3, opt=default_optimization):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.len = l1 + l2 + l3
        self.position = Arm.default_position.copy()
        self.opt = opt

    def get_radians(self, x, y, z):
        distance = sqrt(x ** 2 + y ** 2 + z ** 2)
        if distance > self.len:
            raise Exception("Too far to reach")

        d0 = atan(y / x)
        d1, d2, d3 = self.solve_three(sqrt(x ** 2 + y ** 2), z)
        # return [round(degrees(d0) - 45), round(degrees(d1)), round(degrees(d2)), round(degrees(d3))]
        return d0, d1, d2, d3

    def get_degrees(self, x, y, z):
        self.deg_to_rad(self.get_radians(x, y, z))

    def deg_to_rad(self, rads):
        return [round(degrees(rads[0])), round(degrees(rads[1])), round(degrees(rads[2])), round(degrees(rads[3]))]

    def solve_three(self, a, b):
        rg = self.get_m_range(a, b)
        s = inf
        half_pi = pi / 2
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
        c1 = self.l2 ** 2 + self.l3 ** 2
        c2 = (self.l2 + self.l3) ** 2
        for i in range(-self.l1 * Arm.steps, self.l1 * Arm.steps):
            m = i / Arm.steps
            v = (a - m) ** 2 + (b - sqrt(self.l1 ** 2 - m ** 2)) ** 2
            if c1 < v < c2:
                rg.append(m)
        return rg

    def go_to(self, sr, degs):
        degs = degs.copy()
        degs[0] += 45
        degs[1] = 90 - degs[1]
        degs[2] = 90 - degs[2]
        degs[3] = 90 - degs[3]
        degs.extend(self.position[4:])
        self.position = degs
        print(degs)
        sr.write(self.position)

def get_coordinates(arm, rads):
    d0 = rads[0]
    l1 = arm.l1
    d1 = pi / 2 - rads[1]
    x1, y1 = cos(d1) * l1 * cos(d0), cos(d1) * l1 * sin(d0)
    z1 = sin(d1) * l1
    l2 = arm.l2
    d2 = d1 - rads[2]
    x2, y2 = cos(d2) * l2 * cos(d0) + x1, cos(d2) * l2 * sin(d0) + y1
    z2 = z1 + sin(d2) * l2
    l3 = arm.l3
    d3 = d2 - rads[3]
    x3, y3 = cos(d3) * l3 * cos(d0) + x2, cos(d3) * l3 * sin(d0) + y2
    z3 = z2 + sin(d3) * l3
    return [0, x1, x2, x3], [0, y1, y2, y3], [0, z1, z2, z3]


def callback(e):
    global a, x, y, z, sr, write_serial
    rads = a.get_radians(x, y, z)
    if rads is None:
        return
    degs = a.deg_to_rad(rads)
    print(degs)
    xyz = get_coordinates(a, rads)
    ax.clear()
    ax.set_xlabel('X / mm')
    ax.set_xlim(-50, 200)
    ax.set_ylabel('Y / mm')
    ax.set_ylim(-50, 200)
    ax.set_zlabel('Z / mm')
    ax.set_zlim(0, 200)
    ax.plot(xyz[0], xyz[1], xyz[2], linewidth=2.5, marker='*', markersize=8, markerfacecolor='y')
    ax.scatter(x, y, z, c='r', s=100, marker='o')
    if write_serial:
        a.go_to(sr, degs)


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
    sr = ServoProtocol('COM3')

a = Arm(93, 87, 139, Arm.minimum_change)

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
