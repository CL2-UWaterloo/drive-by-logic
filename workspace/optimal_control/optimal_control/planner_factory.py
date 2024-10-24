#!/usr/bin/python3

from optimal_control.dubins_solver import *
from optimal_control.distributed_dubins_solver import *

from optimal_control.executor import Executor

class PlannerFactory:

    def __init__(self, robot : CarLikeRobot, planner_type : Problem, planner_mode : PlannerMode):
        self.planner : Problem = planner_type(robot, planner_mode)

    def plan(self, init, final, obstacles, number_of_waypoints = 10, granularity = 10, t_max = 120, number_of_agents=1):
        executor = Executor(self.planner)
        executor.prep(
            number_of_agents=number_of_agents,
            number_of_waypoints=number_of_waypoints,
            granularity=granularity, t_max=t_max,
            init=init, final=final, obstacles=obstacles
        )
        return executor.solve(), self.planner


def plan(function, init, final, obstacles, pf, t_max, number_of_waypoints, granularity, number_of_agents):
    # For plotting tools
    pf.planner.final = final
    
    executor = Executor(pf.planner)
    executor.prep(
        number_of_agents=number_of_agents,
        number_of_waypoints=number_of_waypoints,
        granularity=granularity, t_max=t_max,
        init=init, obstacles=obstacles
    )
    function(pf.planner, final, t_max)
    return executor.solve()