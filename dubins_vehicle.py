#!/usr/bin/python3

from dataclasses import dataclass, asdict
from casadi import *

@dataclass
class Coords2D:
    x : float | SX
    y : float | SX
    th : float | SX

@dataclass
class State:
    x : SX
    y : SX
    # Heading Angle
    theta : SX
    # Speed applied at previous state -> Instantaneous Speed
    v : SX
    # Steering angle input applied at previous state
    phi : SX
    # Time elapsed from t=0
    t : SX

class DubinCar:

    def __init__(self) -> None:
        self.coords : Coords2D = Coords2D(0, 0, 0)

        # Limobot params https://github.com/agilexrobotics/limo-doc/blob/master/Limo%20user%20manual(EN).md#13-tech-specifications
        self.minimum_turning_radius = 0.4 # m
        self.wheel_base = 0.2 # m
        self.max_linear_velocity = 1 # m/s
        self.max_acceleration = 0.5 # m/s^2

        self.max_steering_angle = atan2(self.wheel_base, self.minimum_turning_radius)

        self.number_of_states = 15

        self.variables = {}
        self.parameters = {}
        self.defined_parameters = {}

        self.constraints = {} # name : [g, lbg, ubg]

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

    def set_variable(self, name, obj = None):
        if obj != None:
            self.variables[name] = obj
        else:
            self.variables[name] = SX.sym(name)

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

    def prep_problem(self):
        """
        Introduce variables. n states (Including the first and last) therefore, n+1 states.
        """
        self.states : list[State] = []

        # iterate from 0 (initial) to n (final)
        for i in range(self.number_of_states + 1):
            idx = str(i)
            state = State(
                SX.sym("x" + idx), SX.sym("y" + idx), SX.sym("th" + idx),
                SX.sym("v" + idx), SX.sym("phi" + idx), SX.sym("t" + idx)
            )
            for key, value in asdict(state).items():
                self.set_variable(value.name(), value)
            self.states.append(state)

    def prep_constraints(self, init : Coords2D, final : Coords2D):
        self.initial_state = init
        self.final_state = final

        X0 = self.states[0]; Xn = self.states[-1]

        # Boundary Conditions
        self.set_equality_constraint("xi", X0.x, init.x)
        self.set_equality_constraint("yi", X0.y, init.y)
        self.set_equality_constraint("thi", X0.theta, init.th)

        self.set_equality_constraint("xf", Xn.x, final.x, 0.2)
        self.set_equality_constraint("yf", Xn.y, final.y, 0.2)
        self.set_equality_constraint("thf", Xn.theta, final.th)

        # Kinematic Constraints

        # Initial Conditions: start at rest and t = 0
        self.set_equality_constraint("vi", X0.v, 0)
        self.set_equality_constraint("phii", X0.phi, 0)
        self.set_equality_constraint("ti", X0.t, 0)

        # Final Conditions: stop to rest
        self.set_equality_constraint("vf", Xn.v, 0)
        self.set_equality_constraint("phif", Xn.phi, 0)

        # Set kinematics between each state according to simple car
        # https://msl.cs.uiuc.edu/planning/node658.html
        for i in range(1, self.number_of_states+1):
            idx = str(i)
            Xi = self.states[i]; Xi_1 = self.states[i-1]

            # Physical and mathematical constraints

            self.set_constraint("t"+idx, Xi.t, 0)
            self.set_constraint("th_kin"+idx, Xi.theta, -pi, pi)
            self.set_constraint("v_max"+idx, Xi.v, 0, self.max_linear_velocity)
            self.set_constraint("phi_max"+idx, Xi.phi, -self.max_steering_angle, self.max_steering_angle)

            # Bicycle model

            dt = Xi.t - Xi_1.t

            dx = Xi.x - Xi_1.x
            dy = Xi.y - Xi_1.y
            dth = Xi.theta - Xi_1.theta

            self.set_equality_constraint("cycle"+idx, dy*(cos(Xi_1.theta)+cos(Xi.theta)) - dx*(sin(Xi_1.theta)+sin(Xi.theta)), 0)
            self.set_constraint("dt"+idx, dt, 0, 2) # s

            # Velocity and Steering Angle constraints

            d = power(dx, 2) + power(dy, 2)

            self.set_equality_constraint("v"+idx, d - power(Xi.v, 2)*power(dt, 2), 0)
            self.set_equality_constraint("phi"+idx, atan2(dth*self.wheel_base, Xi.v*dt) - Xi.phi, 0)

            # Curvature Constraints

            p = self.wheel_base/tan(Xn.phi)
            self.set_constraint("p_max"+idx, p - self.minimum_turning_radius, 0)

            # Acceleration Constraints

            if (i - 2 >= 0):
                Xi_2 = self.states[i-2]
                dt_1 = Xi_1.t - Xi_2.t
                a = (2*(Xi.v - Xi_1.v))/(dt + dt_1)
                self.set_constraint("a"+idx, a, 0, self.max_acceleration)

        # Time Constraints
        time_sum = 0
        for i in range(0, len(self.states) - 1):
            time_sum += (self.states[i+1].t - self.states[i].t)

        self.set_equality_constraint("time_sum", time_sum - Xn.t + X0.t, 0)

    def objective(self):

        ##### Minimize Path Length
        def length(stateA : State, stateB : State):
            return power((stateA.x - stateB.x), 2) + power((stateA.y - stateB.y), 2)

        path_length = 0
        for i in range(1, len(self.states)):
            path_length += length(self.states[i], self.states[i-1])

        return path_length


        ##### Minimize time of trajectory
        # time_squared_sum = 0
        # for i in range(0, len(self.states) - 1):
        #     time_squared_sum += power((self.states[i+1].t - self.states[i].t), 2)
        
        # return time_squared_sum

    @staticmethod
    def plant(x, y, theta, v, phi, L, dt):
        x += v*cos(theta)*dt
        y += v*sin(theta)*dt
        theta += (v/L)*tan(phi)*dt
        return x, y, theta

    def initial_guess(self):
        """
        Initial guess for the NLP Solver.
        Options:
            1. Use Dubin's Solutions for intial guess.
            2. Build a controller to get an initial guess. (*)
        """
        
        # def plant(v, phi, x, y, theta, L = self.wheel_base):
        #     x += v*cos(theta)
        #     y += v*sin(theta)
        #     theta += (v/L)*tan(phi)

        # def state_position_error(Xi : Coords2D, Xj : Coords2D):
        #     return power(Xi.)

        # guess_states = []
        # current_state = self.initial_state

        guess_variables = []
        initial = 0
        for i in self.get_variables():
            guess_variables.append(DM_rand())
            # if "t" in i.name() and not "h" in i.name():
            #     initial += 1
            #     guess_variables.append(initial)
            # else:
            #     guess_variables.append(1)

        return vertcat(*guess_variables)

    def solve(self, warm_start=None):
        constraints = self.get_constraints()
        nlp = {
            "x" : vertcat(*self.get_variables()),
            "f": self.objective(),
            "g": vertcat(*constraints[0])
        }

        opts = {
            "ipopt": {
                "hessian_approximation": "limited-memory",
                "max_iter": 5000
            },
            "jit": True,
            "compiler": "shell"
        }

        sol = nlpsol("Solver", "ipopt", nlp, opts)

        if warm_start == None:
            return sol(
                x0 = self.initial_guess(),
                lbg = vertcat(*constraints[1]),
                ubg = vertcat(*constraints[2])
            ), sol
        else:
            return sol(
                x0 = warm_start,
                lbg = vertcat(*constraints[1]),
                ubg = vertcat(*constraints[2])
            ), sol

if __name__ == "__main__":
    init = Coords2D(0.0, 0.0, 0.0)
    final = Coords2D(1.0, 1.0, 0.0)

    dc = DubinCar()
    dc.prep_problem()
    dc.prep_constraints(init, final)

    solution, solver = dc.solve()
    # warm_start = solution["x"]

    # for i in range(10):
    #     solution, solver = dc.solve(warm_start)
    #     warm_start = solution["x"]

    # solution, solver = dc.solve(warm_start)
    decision_variables = solution["x"]

    import matplotlib.pyplot as plt

    note_x = []
    note_y = []

    note_phi = []

    for i in range(0, decision_variables.shape[0], 6):
        x = decision_variables[i]; y = decision_variables[i+1]; th = decision_variables[i+2]
        v = decision_variables[i+3]; phi = decision_variables[i+4]; t = decision_variables[i+5]

        print(x, y, th, v, phi, t)

        note_x.append(float(x))
        note_y.append(float(y))
        
        note_phi.append(float(phi))


    plt.plot(note_x, note_y)
    plt.show()