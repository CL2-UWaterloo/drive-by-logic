#!/usr/bin/python3

from dataclasses import dataclass
from casadi import *

@dataclass
class Waypoint:
    X : MX | DM  # State of the robot (6, 1) = (x, y, theta, k, v, a)
    U : MX | DM # Control Inputs to the robot (j, s); s -> rate of change of curvature (2, 1)
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
        return self.X[3]

    @k.setter
    def k(self, value : MX) -> None:
        self.X[3] = value

    @property
    def v(self):
        return self.X[4]

    @v.setter
    def v(self, value : MX) -> None:
        self.X[4] = value

    @property
    def s(self):
        return self.U[1]

    @s.setter
    def s(self, value : MX) -> None:
        self.U[1] = value

    @property
    def a(self):
        return self.X[5]

    @a.setter
    def a(self, value : MX) -> None:
        self.X[5] = value

    @property
    def j(self):
        return self.U[0]

    @j.setter
    def j(self, value : MX) -> None:
        self.U[0] = value

    @staticmethod
    def from_list(value):
        ret = Waypoint(X=DM(vertcat(value[0:6])), U=DM(vertcat(value[6:8])), t=DM(value[8]))
        return ret