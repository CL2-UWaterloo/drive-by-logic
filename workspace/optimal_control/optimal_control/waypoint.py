#!/usr/bin/python3

from dataclasses import dataclass
from casadi import *

@dataclass
class Waypoint:
    X : MX  # State of the robot (4, 1) = (x, y, theta, k)
    U : MX  # Control Inputs to the robot (v, s); s -> rate of change of curvature (2, 1)
    t : MX  # Time taken for transitioning from the previous state to the current state

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
        return self.U[0]

    @v.setter
    def v(self, value : MX) -> None:
        self.U[0] = value

    @property
    def s(self):
        return self.U[1]

    @s.setter
    def s(self, value : MX) -> None:
        self.U[1] = value

