#!/usr/bin/python3

from dataclasses import dataclass
from casadi import *

# Note: Not to be confused with optimization state, only used as syntax sugar
@dataclass
class State:
    x : float
    y : float
    theta : float

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
    max_acceleration : float = 0.5 # m/s^2

def normalize_angle(x):
    x = fmod(x + pi, 2*pi)
    factor = if_else(x < 0, pi, -pi)
    return x + factor

def sinc(x : MX | SX | float):
    if type(x) == SX or type(x) == MX:
        return if_else(x != 0, sin(x)/x, 1)
    else:
        if abs(x) < 1e-20:
            return 1
        else:
            return sin(x)/x

def parametric_arc(k, v, theta, dt):
    phase = k*v*dt*0.5
    arc = v*sinc(phase)*dt
    return arc*cos(theta + phase), arc*sin(theta + phase), 2*phase