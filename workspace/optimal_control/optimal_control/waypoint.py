#!/usr/bin/python3

from dataclasses import dataclass
from typing import ClassVar
from casadi import *

@dataclass
class Waypoint:
    state_size : ClassVar[int] = 4
    input_size : ClassVar[int] = 2
    total_size : ClassVar[int] = state_size + input_size + 1

    X : MX | SX | DM # State of the robot (4, 1) = (x, y, theta, v)
    U : MX | SX | DM # Control Inputs to the robot (a, k)
    t : MX | SX | DM # Time taken for transitioning from the previous state to the current state

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
    def v(self):
        return self.X[3]

    @v.setter
    def v(self, value : MX) -> None:
        self.X[3] = value

    @property
    def a(self):
        return self.U[0]
    
    @a.setter
    def a(self, value : MX) -> None:
        self.U[0] = value

    @property
    def k(self):
        return self.U[1]

    @k.setter
    def k(self, value : MX) -> None:
        self.U[1] = value

    def matrix_form(self):
        return vertcat(self.X, self.U, self.t)

    @staticmethod
    def from_list(value, _castype=SX):
        return Waypoint(
            X=_castype(vertcat(*value[0 : Waypoint.state_size])),
            U=_castype(vertcat(*value[Waypoint.state_size : Waypoint.state_size + Waypoint.input_size])),
            t=_castype(value[Waypoint.state_size + Waypoint.input_size])
        )

    @staticmethod
    def from_vector(vector):
        return Waypoint(
            X = vector[0 : Waypoint.state_size],
            U = vector[Waypoint.state_size : Waypoint.input_size + Waypoint.state_size],
            t = vector[Waypoint.input_size + Waypoint.state_size]
        )

    @staticmethod
    def construct(suffix=""):
        return Waypoint(
            X=SX.sym("X"+suffix, Waypoint.state_size, 1),
            U=SX.sym("U"+suffix, Waypoint.input_size, 1),
            t=SX.sym("t"+suffix, 1, 1)
        )