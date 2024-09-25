#!/usr/bin/python3

from optimal_control.planner_factory import *

def eventually(*expressions, **options):
    lb = options["lb"]; ub = options["ub"]
    with_interval = []
    for expression in expressions:
        window = logic_and(ge(expression[-1].t, lb), ge(ub, expression[-1].t))
        with_interval.append(if_else(window, expression[0], -1e+16))
    return mmax(vertcat(*with_interval))

def always(*expressions, **options):
    lb = options["lb"]; ub = options["ub"]
    with_interval = []
    for expression in expressions:
        window = logic_and(ge(expression[-1].t, lb), ge(ub, expression[-1].t))
        with_interval.append(if_else(window, expression[0], 1e+16))
    return mmin(vertcat(*with_interval))

def eventuallyAlways(*expressions, **options):
    lbo = options["lbo"]; ubo = options["ubo"]
    lbi = options["lbi"]; ubi = options["ubi"]
    with_interval = []
    for expression in expressions:
        window = logic_and(ge(expression[-1].t, lbo), ge(ubo, expression[-1].t))
        lt = expression[-1].t + lbi; ut = expression[-1].t + ubi
        with_interval.append(
            if_else(
                window,
                always(*expressions, lb=lt, ub=ut),
            -1e+16)
        )
    return mmax(vertcat(*with_interval))

def alwaysEventually(*expressions, **options):
    lbo = options["lbo"]; ubo = options["ubo"]
    lbi = options["lbi"]; ubi = options["ubi"]
    with_interval = []
    for expression in expressions:
        window = logic_and(ge(expression[-1].t, lbo), ge(ubo, expression[-1].t))
        lt = expression[-1].t + lbi; ut = expression[-1].t + ubi
        with_interval.append(
            if_else(
                window,
                eventually(*expressions, lb=lt, ub=ut),
            1e+16)
        )
    return mmin(vertcat(*with_interval))

# To this class add STL functions such as Always, Eventually, Until, Boolean (Grammar)
# Also add Smooth Max and Smooth Min to the same as static methods?
class STLPlanner:

    def __init__(self, planner_factory : PlannerFactory):
        self.planner_factory = planner_factory

    def plan(self, init, final, obstacles, number_of_waypoints = 10, granularity = 10, t_max = 120):
        self.planner_factory.planner.final = final
        executor = Executor(self.planner_factory.planner)
        executor.prep(
            number_of_waypoints=number_of_waypoints,
            granularity=granularity, t_max=t_max,
            init=init, obstacles=obstacles
        )
        self.set_objective(final, t_max)
        return executor.solve(), self.planner_factory.planner

    def set_objective(self, final, t_max):
        # Write STL objective in this function.
        # Can add minimum robustness constraint as well.
        goal_signals = []
        for goal in final:
            reach_signals = []
            for signal in self.planner_factory.planner.signals:
                dx = power(signal.x - goal[0], 2)
                dy = power(signal.y - goal[1], 2)
                reach_signals.append(
                    (-(dx + dy - 4), signal)
                )
            goal_signals.append(
                eventually(*reach_signals, lb=0, ub=t_max)
            )

        obstacle_signals = []
        for obstacle in self.planner_factory.planner.obstacles:
            for signal in self.planner_factory.planner.signals:
                dx = power(signal.x - obstacle[0], 2)
                dy = power(signal.y - obstacle[1], 2)
                obstacle_signals.append(
                    always(
                        (dx + dy - power(obstacle[2], 2), signal), 
                        lb=0, ub=t_max
                    )
                )

        self.planner_factory.planner.cost = mmin(
            vertcat(
                *goal_signals,
                *obstacle_signals
            )
        )

        self.planner_factory.planner.set_constraint("robustness", self.planner_factory.planner.cost, lbg=0.25, ubg=4.0)
        self.planner_factory.planner.cost *= -1 # Minimize negative robustness

        self.planner_factory.planner.cost += 0.1*self.planner_factory.planner.time_elapsed