#!/usr/bin/python3

from optimal_control.problem import Problem

class Executor:

    def __init__(self, problem : Problem):
        self.problem = problem
    
    def prep(self, *args, **kwargs):
        self.problem.prep_problem(*args, **kwargs)
        self.problem.prep_constraints(*args, **kwargs)
    
    def solve(self, *args, **kwargs):
        if "warming_iterations" in kwargs.keys():
            result = self.problem.solve(iterations=kwargs["warming_iterations"])
            return self.problem.solve(iterations=500, warm_start=result)
        else:
            return self.problem.solve(iterations=500)