#!/usr/bin/python3

from dataclasses import dataclass
from casadi import *

@dataclass
class Waypoint:
    X : MX | DM # State of the robot (3, 1) = (x, y, theta)
    U : MX | DM # Control Inputs to the robot (v, k); s -> rate of change of curvature (2, 1)
    t : MX | DM # Time taken for transitioning from the previous state to the current state

    @property
    def x(self):
        return self.X[0]
    
    @x.setter
    def x(self, value : MX) -> None:
        self.X[0] = value

    @property
    def y(self):
        return self.X[1]

    @y.setter
    def y(self, value : MX) -> None:
        self.X[1] = value

    @property
    def theta(self):
        return self.X[2]

    @theta.setter
    def theta(self, value : MX) -> None:
        self.X[2] = value

    @property
    def k(self):
        return self.U[1]

    @k.setter
    def k(self, value : MX) -> None:
        self.U[1] = value

    @property
    def v(self):
        return self.U[0]

    @v.setter
    def v(self, value : MX) -> None:
        self.U[0] = value

    @staticmethod
    def from_list(value):
        ret = Waypoint(X=DM(vertcat(value[0:3])), U=DM(vertcat(value[3:5])), t=DM(value[6]))
        return ret

    def matrix_form(self):
        return vertcat(self.X, self.U, self.t)