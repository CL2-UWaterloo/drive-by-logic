#!/usr/bin/python3

from optimal_control.problem import Problem

class Executor:

    def __init__(self, problem : Problem):
        self.problem = problem
    
    def prep(self, *args, **kwargs):
        self.problem.prep_problem(*args, **kwargs)
        self.problem.prep_constraints(*args, **kwargs)
    
    def solve(self, *args, **kwargs):
        iterations = 5000
        if "max_iters" in kwargs.keys():
            iterations = kwargs["max_iters"]

        if "warming_iterations" in kwargs.keys():
            result, _ = self.problem.solve(iterations=kwargs["warming_iterations"])
            return self.problem.solve(iterations=iterations, warm_start=result["x"])
        elif "initial_guess" in kwargs.keys():
            return self.problem.solve(iterations=iterations, warm_start=kwargs["initial_guess"])
        else:
            return self.problem.solve(iterations=iterations)

def prep(init, planner, hrz, number_of_waypoints, granularity, number_of_agents):
    if type(init) != list:
        init = [init]
    executor = Executor(planner)
    executor.prep(
        number_of_agents=number_of_agents,
        number_of_waypoints=number_of_waypoints,
        granularity=granularity, hrz=hrz,
        init=init
    )
    return executor