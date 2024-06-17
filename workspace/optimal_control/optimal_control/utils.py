#!/usr/bin/python3

from dataclasses import dataclass
from casadi import *

from rclpy.node import Node
from geometry_msgs.msg import Twist

from optimal_control.waypoint import Waypoint

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

    def get_state_space(self, xnm1 : Waypoint, xn : Waypoint, dt : MX | float):
        B = MX.zeros(xnm1.X.shape[0], xn.U.shape[0])

        B[0, 0] = cos(xnm1.theta)*dt; B[0, 1] = 0.0
        B[1, 0] = sin(xnm1.theta)*dt; B[1, 1] = 0.0
        B[2, 0] = xnm1.k*dt; B[2, 1] = xn.v*(dt**2)
        B[3, 0] = dt; B[3, 1] = 0.0

        # MX.eye = Identity Matrix
        return MX.eye(xnm1.X.shape[0]), B 

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

class TwistPublisher(Node):

    def __init__(self, dt, commands):
        super().__init__("twist_publisher")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.commands = commands
        self.timer = self.create_timer(dt, self.callback)
        self.index = 0

    def callback(self):
        msg = Twist()
        msg.linear.x = self.commands[self.index][0]
        msg.angular.z = self.commands[self.index][1]
        self.publisher.publish(msg)
        self.index += 1