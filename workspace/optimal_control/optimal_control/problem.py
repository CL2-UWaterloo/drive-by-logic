#!/usr/bin/python3

from casadi import *

class Problem:

    def __init__(self):
        self.variables = {}
        self.parameters = {}
        self.defined_parameters = {}

        self.constraints = {} # name : [g, lbg, ubg]

        self.obstacles = []

    def set_constraint(self, name, exp, lbg=-DM_inf(), ubg=DM_inf()):
        self.constraints[name] = [exp, lbg, ubg]

    def set_equality_constraint(self, name, exp, value, abs_tol=1e-5):
        self.constraints[name] = [exp, value - abs_tol, value + abs_tol]
    
    def get_constraints(self):
        g = []; lbg = []; ubg = []
        for name in self.constraints.keys():
            constraint = self.constraints[name]
            g.append(constraint[0])
            lbg.append(constraint[1])
            ubg.append(constraint[2])

        return g, lbg, ubg

    def get_constraint_idx_by_pattern(self, pattern):
        idx = 0; ret = []
        for name in self.constraints.keys():
            if pattern in name:
                ret.append(idx)
            idx += 1
        return ret

    def set_variable(self, name, obj = None):
        if obj != None:
            self.variables[name] = obj
        else:
            self.variables[name] = MX.sym(name)

    def get_variable(self, name):
        return self.variables[name]

    def get_variables(self):
        variables = []
        for name in self.variables.keys():
            variables.append(self.variables[name])
        return variables

    def set_defined_parameters(self, name, value):
        self.defined_parameters[name] = value

    def get_defined_parameters(self, name):
        return self.defined_parameters[name]

    ##### Abstract Functions

    def prep_problem(self, *args, **kwargs):
        pass

    def prep_constraints(self, *args, **kwargs):
        pass

    def objective(self, *args, **kwargs):
        pass

    def initial_guess(self, *args, **kwargs):
        pass

    def solve(self, *args, **kwargs):
        pass