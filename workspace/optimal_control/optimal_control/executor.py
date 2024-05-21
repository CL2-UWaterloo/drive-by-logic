#!/usr/bin/python3

from optimal_control.problem import Problem

class Executor:

    def __init__(self, problem : Problem):
        self.problem = problem
    
    def prep(self, *args, **kwargs):
        self.problem.prep_problem(*args, **kwargs)
        self.problem.prep_constraints(*args, **kwargs)

    def solve(self, *args, **kwargs):
        solution, solver = self.problem.solve()
        if "warm_start" in kwargs.keys():
            warm_start = solution["x"]
            for _ in range(kwargs["warming_iterations"] - 1):
                solution, solver = self.problem.solve(warm_start = warm_start)
                warm_start = solution["x"]
            solution, solver = self.problem.solve(warm_start = warm_start)
        return solution, solver