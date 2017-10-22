from sympy import *
import sympy
import numpy as np
import math

# i = Symbol('i')
# j = Symbol('j')
# m = Symbol('m')
# n = Symbol('n')

# a = Symbol('a')
# b = Symbol('b')
# l1 = Symbol('l1')
# A = Symbol('A')

a = 30
b = 0
l1 = 10
l2 = 10
l3 = 10
A = a ** 2 + b ** 2 + l1 ** 2 - l2 ** 2 - l3 ** 2
m = Symbol('m')
# print(simplify(solve([(m - i) ** 2 + (n - j) ** 2 - l1 ** 2, (a - i) ** 2 + (b - j) ** 2 - l2 ** 2], [i, j])))


# a, m, b, l1, l2, l3 = sympy.symbols('a m b l1 l2 l3')
# solve_poly_inequality(Poly((a - m) ** 2 + (b - sqrt(l1 ** 2 - m ** 2)) ** 2 - l2 ** 2 - l3 ** 2, m, domain='ZZ'),
#                       '>=')

# print(solve_poly_inequality(Poly(m ** 4 + 4 * a * m ** 3 + (
#     4 * a ** 2 + 4 * b ** 2 - 2 * A) * m ** 2 - 4 * a * A * m - 4 * b ** 2 * l1 ** 2 + A ** 2, m, domain='ZZ'),
# #                       '>='))

print(np.roots([1, -4 * a, 4 * a ** 2 + 4 * b ** 2 + 2 * A, - 4 * a * A, - 4 * b ** 2 * l1 ** 2 + A ** 2]))
print(solve(m**4 -4 * a*m**3+ (4 * a ** 2 + 4 * b ** 2 + 2 * A)*m**2 - 4 * a * A*m - 4 * b ** 2 * l1 ** 2 + A ** 2, m))