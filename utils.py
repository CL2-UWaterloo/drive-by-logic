#!/usr/bin/python3

from dataclasses import dataclass
from casadi import *

@dataclass
class Coords2D:
    x : float | SX
    y : float | SX
    th : float | SX

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