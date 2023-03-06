import control
import matplotlib.pyplot as plt
import numpy as np
import odrive
import time
import IMU
import matplotlib.pyplot as plt

from scipy.integrate import ode
from odrive.enums import *
from odriveDriver import Axis


TURNS_TO_METERS = 0.528

IMU = IMU.IMU(0, 40)
IMU.setupIMU()

odrv0 = odrive.find_any()
axis0 = Axis(odrv0.axis0)
axis1 = Axis(odrv0.axis1, dir=-1)

axis0.setup()
axis1.setup()

class LQG:
    MATLAB_OUTPUT_DIR = './'

    def __init__(self):
        self.A = np.loadtxt(MATLAB_OUTPUT_DIR + 'A.txt')
        self.B = np.loadtxt(MATLAB_OUTPUT_DIR + 'B.txt')

        self.init_pos = axis0.get_pos_turns() * TURNS_TO_METERS

        theta0 = self.measure()[2]
        x0 = np.array([0,0,theta,0])
        self.kf = KalmanFilter(A, B, x0=x0)
        self.lqr = LQR(A, B, mode='position')
        self.u = 0

    def calculate_torque(self, t):
        y = self.measure()
        x_hat = self.kf.estimate(t, self.u, y)
        u = self.lqr.calculate_torque(x_hat)
        self.u = u
        # LQR calculates total torque, u/2 = torque per motor
        return u/2

    def measure(self):
        x = axis0.get_pos_turns() * TURNS_TO_METERS - self.init_pos
        v = axis0.get_vel() * TURNS_TO_METERS
        theta = IMU.getAngleCompRads()
        y = np.array([x, v, theta])
        return y
    

class LQR:
    def __init__(self, A, B):
        self.A = A
        self.B = B
        self.calculate_gains()
        self.set_target(x=0)

    def set_target(x=None, v=None):
        if x is None and y is None:
            raise ValueError('ERROR: No target selected!! Set x or v in the target')

        if x is not None:
            self.target = np.array([x, 0, 0, 0])
            self.mode = 'position'
        if v is not None:
            self.target = np.array([0, v, 0, 0])
            self.mode = 'velocity'
            print('WARNING: Velocity mode not implemented yet')

    def calculate_gains(self, mode='position'):
        Q = np.diag([100, 1, 1, 1])
        R = 0.1

        self.Kx, _, _ = control.lqr(self.A, self.B, Q, R)
        # TODO implement velocity target
        self.Kv = self.Kx

    def calculate_torque(self, x):
        K = self.Kx if self.mode=='position' else self.Kv
        err = x - self.target
        u = -K @ err
        return u


class KalmanFilter:
    def __init__(self, A, B, x0=np.zeros(4)):
        # We measure only the first 3 state variables (missing wogma)
        C = np.eye(4)[:3]
        # TODO: Determine these from noise amplitude
        Vn = np.diag([0.1, 0.1, 0.1, 0.1])
        Vd = 1
        self.K, _, _ = lqe(A, Vd, C, Vd, Vn)
        self.x_hat = x0
        self.t = 0
        self.history = {'y': [], 'x_hat': []}
    
    def update(t, x_hat, u, y):
        return (self.A - self.K @ self.C) @ self.x_hat + self.K @ y + self.B @ u

    def estimate(self, t, u, y):
        x_hat = solve_ivp(update, tspan=(self.t, t), y0=self.x_hat, args=(u,y))
        self.t = t
        self.history['y'].append(y)
        self.history['x_hat'].append(x_hat)
        return x_hat


def brake_both_motors(axis0, axis1):
    while abs(axis0.get_vel()) > 0.05 and abs(axis1.get_vel()) > 0.05:
        axis0.set_trq(-0.5 if axis0.get_vel() > 0 else 0.5)
        axis1.set_trq(-0.5 if axis1.get_vel() > 0 else 0.5)
        
    axis0.set_trq(0)
    axis1.set_trq(0)


if __name__ == '__main__':
    lqg = LQG()

    t = 0
    t0 = time.time()
    while t < 10:
        t = time.time() - t0
        torque = lqg.calculate_torque(t)
        axis0.set_trq(torque)
        axis1.set_trq(torque)
    
    brake_both_motors(axis0, axis1)

    y = np.stack(lqg.kf.history['y'], axis=-1)
    x_hat = np.stack(lqg.kf.history['x_hat'], axis=-1)
    for i, l, c in zip(range(4), ['x', 'v', 'θ', 'ω'], ['tab:orange', 'tab:red', 'tab:green', 'tab:blue']):
        if i < 3:
            plt.plot(y[i], label=l+' - measured', linestyle='-', c=c)
        plt.plot(x_hat[i], label=l+' - predicted', linestyle='--', c=c)
    plt.legend()