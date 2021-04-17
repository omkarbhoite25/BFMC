import math
import numpy as np
import bisect
import matplotlib.pyplot as plt


class Spline:
    """
    Cubic Spline class
    """

    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)

        # calc coefficient c
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)
        #  print(self.c1)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        """
        Calc position

        if t is outside of the input x, return None

        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return result

    def calcd(self, t):
        """
        Calc first derivative

        if t is outside of the input x, return None
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        """
        Calc second derivative
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def __search_index(self, x):
        """
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        #  print(A)
        return A

    def __calc_B(self, h):
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        return B


class Spline2D:
    """
    2D Cubic Spline class

    """

    def __init__(self):
        self.ax = None
        self.ay = None
        self.s = None
        self.sx = None
        self.sy = None
        self.rx = None
        self.ry = None
        self.ryaw = None
        self.rk = None
        self.s = None

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)

        return x, y

    def calc_curvature(self, s):
        """
        calc curvature
        """
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        return k

    def calc_yaw(self, s):
        """
        calc yaw
        """
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = math.atan2(dy, dx)
        return yaw

    def calc_spline_course(self, ax=None, ay=None, ds=0.1):
        """ 
        calc continues path given waypoints
        """

        self.ax = ax
        self.ay = ay

        s = self.__calc_s(ax, ay)
        self.sx = Spline(s, self.ax)
        self.sy = Spline(s, self.ay)
        self.s = list(np.arange(0, s[-1], ds))

        self.rx, self.ry, self.ryaw, self.rk = [], [], [], []
        for i_s in self.s:
            ix, iy = self.calc_position(i_s)
            self.rx.append(ix)
            self.ry.append(iy)
            self.ryaw.append(self.calc_yaw(i_s))
            self.rk.append(self.calc_curvature(i_s))

    def get_spline_path(self):
        """
        return the computed path
        """
        return self.rx, self.ry, self.ryaw, self.rk, self.s 

    def plot_spline_path(self):
        """
        plot path
        """

        plt.subplots(1)
        plt.plot(self.ax, self.ay, "xb", label="input")
        plt.plot(self.rx, self.ry, "-r", label="spline")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.subplots(1)
        plt.plot(self.s, [np.rad2deg(iyaw) for iyaw in self.ryaw], "-r", label="yaw")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("yaw angle[deg]")

        plt.subplots(1)
        plt.plot(self.s, self.rk, "-r", label="curvature")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("curvature [1/m]")

        plt.show()



def main():  
    print("Spline 2D test")
    x = [2.22, 1.9, 1.57, 1.26, 1.01, 0.88, 0.86, 0.86, 0.9, 1.1, 1.38, 1.7, 2.02, 2.22, 2.97, 2.79, 2.79, 2.79, 2.79, 2.98, 3.74, 4.06, 4.38, 4.69, 4.95, 5.08, 5.1, 5.1, 5.04, 4.89, 4.6, 4.28, 3.96, 3.73, 2.98, 3.17, 3.16, 3.16, 2.98]
    y = [2.8, 2.81, 2.81, 2.76, 2.55, 2.26, 1.94, 1.62, 1.3, 1.04, 0.89, 0.87, 0.88, 0.87, 0.69, 1.43, 1.75, 2.07, 2.23, 2.99, 3.17, 3.18, 3.18, 3.23, 3.42, 3.71, 4.03, 4.35, 4.66, 4.94, 5.09, 5.1, 5.11, 5.1, 5.29, 4.54, 4.22, 3.74, 2.99]
    ds = 0.7  # [m] distance of each intepolated points

    sp = Spline2D()
    sp.calc_spline_course(x, y, ds)
    sp.plot_spline_path()

    

if __name__ == '__main__':
    main()
