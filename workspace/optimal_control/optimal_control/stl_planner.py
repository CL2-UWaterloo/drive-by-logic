#!/usr/bin/python3

from optimal_control.planner_factory import *

# To this class add STL functions such as Always, Eventually, Until, Boolean (Grammer)
# Also add Smooth Max and Smooth Min to the same as static methods?
class STLPlanner:

    def __init__(self, planner_factory : PlannerFactory):
        self.planner_factory = planner_factory
        self.set_objective()

    def plan(self, init, final, obstacles, number_of_waypoints = 10, granularity = 10, t_max = 120):
        executor = Executor(self.planner_factory.planner)
        executor.prep(
            number_of_waypoints=number_of_waypoints,
            granularity=granularity, t_max=t_max,
            init=init, final=final, obstacles=obstacles
        )
        self.set_objective()
        return executor.solve(), self.planner_factory.planner

    def set_objective(self):
        # Write STL objective in this function.
        # Can add minimum robustness constraint as well.
        goal_signals = []
        for signal in self.planner_factory.planner.signals:
            dx = signal.x - self.planner_factory.planner.final_state.x
            dy = signal.y - self.planner_factory.planner.final_state.y
            mdx = self.planner_factory.planner.final_state.x - signal.x
            mdy = self.planner_factory.planner.final_state.y - signal.y
            goal_signals.append(STLPlanner.smooth_min(
                0.1-dx, 0.1-dy, 0.1-mdx, 0.1-mdy, k = 60
            ))
        self.planner_factory.planner.cost = STLPlanner.smooth_max(*goal_signals, k = 60)
        self.planner_factory.planner.cost *= -1 # Maximize negative robustness

    @staticmethod
    def smooth_max(*expressions, k = 20):
        expsum = 0
        for expression in expressions:
            expsum += exp(k*expression)
        return log(expsum)*(1/k)

    @staticmethod
    def smooth_min(*expressions, k = 20):
        expsum = 0
        for expression in expressions:
            expsum += exp(-k*expression)
        return -log(expsum)*(1/k)

    @staticmethod
    def eventually(*expressions, k = 20):
        return STLPlanner.smooth_max(*expressions, k)