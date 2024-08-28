#!/usr/bin/python3

from optimal_control.dubins_planner import *
from optimal_control.smooth_planner import *
from optimal_control.executor import Executor

class PlannerFactory:

    def __init__(self, robot : CarLikeRobot, planner_type : Problem, planner_mode : PlannerMode):
        self.planner : Problem = planner_type(robot, planner_mode)

    def plan(self, init, final, obstacles, number_of_waypoints = 10, granularity = 10, t_max = 120):
        executor = Executor(self.planner)
        executor.prep(
            number_of_waypoints=number_of_waypoints,
            granularity=granularity, t_max=t_max,
            init=init, final=final, obstacles=obstacles
        )
        return executor.solve(), self.planner