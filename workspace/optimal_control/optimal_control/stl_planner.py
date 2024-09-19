#!/usr/bin/python3

from optimal_control.planner_factory import *

def sigmoid(x):
    return 1/(1 + exp(-x))

def smooth_min(*expressions):
    k = -20; expsum = 0
    for expression in expressions:
        expsum += exp(k*expression)
    return (1/k)*(log(expsum))

def smooth_max(*expressions):
    k = 20; expsum = 0
    for expression in expressions:
        expsum += exp(k*expression)
    return (1/k)*(log(expsum))

def eventually(*expressions, **options):
    lb = options["lb"]; ub = options["ub"]; dt = options["dt"]
    with_interval = []
    i = int(floor(lb/dt)); j = int(floor(ub/dt))
    print(len(expressions), i, j, dt)
    # Expected expression format: (formula, source signal)
    for idx in range(i, j):
        with_interval.append(expressions[idx])
    return smooth_max(*with_interval)

def always(*expressions, **options):
    lb = options["lb"]; ub = options["ub"]; dt = options["dt"]
    with_interval = []
    i = int(floor(lb/dt)); j = int(floor(ub/dt))
    # Expected expression format: (formula, source signal)
    for idx in range(i, j):
        with_interval.append(expressions[idx])
    return smooth_min(*with_interval)

# To this class add STL functions such as Always, Eventually, Until, Boolean (Grammar)
# Also add Smooth Max and Smooth Min to the same as static methods?
class STLPlanner:

    def __init__(self, planner_factory : PlannerFactory):
        self.planner_factory = planner_factory

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
        reach1_signals = []
        for signal in self.planner_factory.planner.signals:
            goal_lb_x = self.planner_factory.planner.final_state.x - 1; goal_ub_x = self.planner_factory.planner.final_state.x + 1
            goal_lb_y = self.planner_factory.planner.final_state.y - 1; goal_ub_y = self.planner_factory.planner.final_state.y + 1
            # Goal Signal = (*Formula, Signal)
            reach1_signals.append(
                (
                    smooth_min(
                        signal.x - goal_lb_x,
                        signal.y - goal_lb_y,
                        goal_ub_x - signal.x,
                        goal_ub_y - signal.y
                    )
                )
            )

        reach2_signals = []
        for signal in self.planner_factory.planner.signals:
            goal_lb_x = 0 - 1; goal_ub_x = 0 + 1
            goal_lb_y = 10 - 1; goal_ub_y = 10 + 1
            # Goal Signal = (*Formula, Signal)
            reach2_signals.append(
                (
                    smooth_min(
                        signal.x - goal_lb_x, 
                        signal.y - goal_lb_y, 
                        goal_ub_x - signal.x, 
                        goal_ub_y - signal.y
                    )
                )
            )

        avoid_signals = []
        for signal in self.planner_factory.planner.signals:
            obstacle_signals = []
            for obstacle in self.planner_factory.planner.obstacles:
                dx = power(signal.x - obstacle[0], 2)
                dy = power(signal.y - obstacle[1], 2)
                obstacle_signals.append(
                    sqrt(dx + dy) - obstacle[2] - obstacle[3]
                )
            avoid_signals.append(
                smooth_min(*obstacle_signals)
            )

        T = self.planner_factory.planner.t_max
        N = self.planner_factory.planner.number_of_waypoints
        M = self.planner_factory.planner.granularity
        dt = (T/((N-1)*M))
        self.planner_factory.planner.cost = smooth_min(
            eventually(*reach1_signals, lb=0, ub=80, dt=dt),
            eventually(*reach2_signals, lb=100, ub=150, dt=dt),
            always(*avoid_signals, lb=0, ub=150, dt=dt)
        )

        self.planner_factory.planner.cost *= -1 # Minimize negative robustness