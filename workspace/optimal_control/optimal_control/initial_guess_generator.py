from optimal_control.utils import *
from optimal_control.environment import *
from optimal_control.plotter import *

"""
Initial Guess Generator Test

Inputs: X1, X2, ..., XN states to connect with each other.
Outputs: decision variables for the same.

Opt Prog: Glorified Feasibility Check really

min_{decision vars} 1
s.t. curve constraints
    state constraints

additionally, the only extra design constraint: Have the same decision variables
"""

# Number of Waypoints also includes the starting point
def generate_guess(
        initial_state : State,
        states : list[State],
        number_of_waypoints : int,
        robot : CarLikeRobot,
        total_time : float
    ):
    waypoints = []; variables = []
    parameters = []; parameter_values = []; 
    constraints = []; constraints_lbg = []; constraints_ubg = []

    # Construction of decision variables and expressions
    X0 = Waypoint.construct("0")
    parameters.append(X0.matrix_form())
    parameter_values.append(vertcat(
        initial_state.x, initial_state.y, initial_state.theta,
        initial_state.v, 0.0, 0.0, 0.0
    ))
    waypoints.append(X0)

    max_acceleration = robot.get_max_acceleration()
    max_curvature = 1/robot.get_minimum_turning_radius()
    max_velocity = robot.get_max_linear_velocity()

    signals : list[Waypoint] = []; time_sum = 0
    for i in range(1, number_of_waypoints):
        idx = str(i)
        Xi = Waypoint.construct(idx); Xim1 = waypoints[i-1]
        variables.append(Xi.matrix_form()); waypoints.append(Xi)

        # Adding constraints for inputs
        constraints.append(Xi.a)
        constraints_lbg.append(-max_acceleration)
        constraints_ubg.append(max_acceleration)

        constraints.append(Xi.k)
        constraints_lbg.append(-max_curvature)
        constraints_ubg.append(max_curvature)

        constraints.append(Xi.v)
        constraints_lbg.append(0)
        if i != number_of_waypoints -1:
            constraints_ubg.append(max_velocity)
        else:
            constraints_ubg.append(0)

        # Making Constraints: Manipulate M here to ensure numeric feasibility
        dx = Xim1.x; dy = Xim1.y; dth = Xim1.theta; dv = Xim1.v
        M = 20; dt = Xi.t/M
        for j in range(M):
            dv += Xi.a*dt

            px, py, pth = parametric_arc(Xi.k, dv, dth, dt)
            dx += px; dy += py; dth += pth

            signals.append(
                Waypoint.from_list(
                    [dx, dy, dth, dv, Xi.a, Xi.k, (j+1)*dt]
                )
            )
        
        diff = Xi.X - vertcat(dx, dy, dth, dv)

        # Add Equality Constraint
        constraints.append(diff)
        constraints_lbg.append(GenDM_zeros(4,1))
        constraints_ubg.append(constraints_lbg[-1])

        time_sum += Xi.t

    constraints.append(time_sum)
    constraints_lbg.append(total_time)
    constraints_ubg.append(total_time)

    for state in states:
        predicates = []
        for signal in signals:
            # If dxpdy is > ep it is positive; If dxpdy is = ep it is 0; else negative. First Clamp it. How? Relu it
            dxpdyme = fmax(power(signal.x - state.x, 2) + power(signal.y - state.y, 2), 0)
            
            # Now add to the predicates
            predicates.append(dxpdyme)
        
        # Get the formula. This has to be zero.
        formula = mmin(vertcat(*predicates))
        
        # Now add the constraint
        constraints.append(formula)
        constraints_lbg.append(0)
        constraints_ubg.append(0)

    # For solving the problem
    nlp = {
        "x" : vertcat(*variables),
        "f" : 1.0,
        "g" : vertcat(*constraints),
        "p" : vertcat(*parameters)
    }

    opts = {
        "ipopt" : {
            "hessian_approximation": "limited-memory",
            "max_iter": 500,
            "tol": 1e-2
        },
        "print_time": False,
        "expand": True
    }

    sol = nlpsol("Solver", "ipopt", nlp, opts)

    solution = sol(
        lbg = vertcat(*constraints_lbg),
        ubg = vertcat(*constraints_ubg),
        p = vertcat(*parameter_values)
    )

    return solution, sol