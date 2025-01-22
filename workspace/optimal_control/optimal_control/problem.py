#!/usr/bin/python3

from casadi import *

class Problem:

    def __init__(self):
        # Optimization Variables
        self.variables = {}

        # Parameters for equality constraints (initial states and such)
        self.parameter_symbols = {}
        self.parameter_values = {}

        # name : [g, lbg, ubg]
        self.constraints = {}

        # Objective function
        self.cost = 1

        # Signal Storage
        # Stores symbolic states of the trajectory
        self.signals = []

    def set_constraint(self, name, exp, lbg=-DM.inf(), ubg=DM.inf()):
        self.constraints[name] = [exp, lbg, ubg]

    def set_equality_constraint(self, name, exp, value):
        self.constraints[name] = [exp, value, value]

    def get_constraints(self):
        g = []; lbg = []; ubg = []

        for name in self.constraints.keys():
            constraint = self.constraints[name]
            g.append(constraint[0])
            lbg.append(constraint[1])
            ubg.append(constraint[2])

        return g, lbg, ubg

    def set_variable(self, name, obj = None):
        if obj != None:
            self.variables[name] = obj
        else:
            self.variables[name] = SX.sym(name)

    def get_variable(self, name):
        return self.variables[name]

    def get_variables(self):
        return list(self.variables.values())

    def set_parameter(self, name, symbol, value):
        self.parameter_symbols[name] = symbol
        self.parameter_values[name] = value

    def get_parameter_symbol(self, name):
        return self.parameter_symbols[name]

    def get_parameter_value(self, name):
        return self.parameter_values[name]

    def get_parameters_symbols(self):
        return list(self.parameter_symbols.values())

    def get_parameters_values(self):
        return list(self.parameter_values.values())

    def objective(self, *args, **kwargs):
        return self.cost

    ##### Abstract Functions

    # TODO: Use ABC for this
    def prep_problem(self, *args, **kwargs):
        pass

    def prep_constraints(self, *args, **kwargs):
        pass

    def initial_guess(self, *args, **kwargs):
        pass

    def solve(self, *args, **kwargs):
        pass