#!/usr/bin/python3

from typing import Tuple
from enum import Enum

from dataclasses import dataclass
from casadi import *

from optimal_control.waypoint import *

from math import factorial

MARGIN = DM([1e-2])
INF = SX_inf()

# Note: Not to be confused with optimization state, only used as syntax sugar
@dataclass
class State:
    x : float | MX | DM
    y : float | MX | DM
    theta : float | MX | DM
    v : float | MX | DM

def cast_state(state : State, _casttype : float | MX | DM = float):
    return State(
        _casttype(state.x), _casttype(state.y),
        _casttype(state.theta), _casttype(state.v)
    )

@dataclass
class CarLikeRobot:
    minimum_turning_radius : float
    wheel_base : float
    max_linear_velocity : float
    max_acceleration : float

    def get_max_steering_angle(self):
        return atan2(self.wheel_base, self.minimum_turning_radius)

    def get_minimum_turning_radius(self):
        return self.minimum_turning_radius
    
    def get_wheel_base(self):
        return self.wheel_base

    def get_max_linear_velocity(self):
        return self.max_linear_velocity

    def get_max_acceleration(self):
        return self.max_acceleration

# Limobot params https://github.com/agilexrobotics/limo-doc/blob/master/Limo%20user%20manual(EN).md#13-tech-specifications
@dataclass
class LimoBot(CarLikeRobot):
    minimum_turning_radius : float = 0.4 # m
    wheel_base : float = 0.2 # m
    max_linear_velocity : float = 1 # m/s
    max_acceleration : float = 0.2 # m/s^2

def normalize_angle(x):
    x = fmod(x + pi, 2*pi)
    factor = if_else(x < 0, pi, -pi)
    return x + factor

def sinc(x : MX | SX | float, n = 8):
    tsum = 1
    for i in range(1, n+1):
        fac = factorial((2*n)+1)
        tsum += power(-1, n) * power(x, 2*n)/fac
    return tsum

# def sinc(x : MX | SX | float):
#     if type(x) == SX or type(x) == MX:
#         return if_else(x != 0, sin(x)/x, 1)
#     else:
#         if abs(x) < MARGIN:
#             return 1
#         else:
#             return sin(x)/x

def parametric_arc(k, v, theta, dt):
    phase = k*v*dt*0.5
    arc = v*sinc(phase)*dt
    return arc*cos(theta + phase), arc*sin(theta + phase), 2*phase

class PlannerMode(Enum):
    ClosedForm = 0
    ForwardSim = 1

def get_state_from_waypoint(value : Waypoint):
    return State(value.x, value.y, value.theta, value.v)

@dataclass
class PlotOptions:
    xlim : Tuple[float, float]
    ylim : Tuple[float, float]
    title : str