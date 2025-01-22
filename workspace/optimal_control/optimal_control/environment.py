from copy import deepcopy
from typing import Tuple

# Need this for casadi
from optimal_control.utils import *

# Need for plot
import matplotlib.patches as patches

# Need this for abstraction. TODO: Use ABC
class Region:

    def inside_region(self, state : State) -> float | MX | DM:
        pass

    def outside_region(self, state : State) -> float | MX | DM:
        pass

    def set_patch_color(self, color : str ="red"):
        self.patch.set_edgecolor(color)

    def plot(self) -> patches.Patch:
        return deepcopy(self.patch)

class Circle(Region):

    def __init__(self, radius : float, center : Tuple[float, float], color : str = "red"):
        self.radius = radius
        self.center = center
        self.patch = patches.Circle(
            self.center,
            self.radius,
            linewidth=1, edgecolor=color, facecolor='none'
        )

    def inside_region(self, state : State) -> float | MX | DM:
        dx = state.x - self.center[0]
        dy = state.y - self.center[1]

        # We want radius - distance > 0
        return power(self.radius, 2) - power(dx, 2) - power(dy, 2)

    def outside_region(self, state) -> float | MX | DM:
        dx = state.x - self.center[0]
        dy = state.y - self.center[1]

        # We want distance - radius > 0
        return power(dx, 2) + power(dy, 2) - power(self.radius, 2)

class Rectangle(Region):
    """
    +------------------------+
    |                        |
    h         (x,y)          |
    |                        |
    +---------- w -----------+
    """
    def __init__(self,
        height : float, width : float,
        center : Tuple[float, float],
        color : str = "red"
    ):
        self.height = height
        self.width = width
        self.center = center

        self.anchor = (self.center[0] - width/2, self.center[1] - height/2)
        self.patch = patches.Rectangle(
            self.anchor,
            self.width,
            self.height,
            linewidth=1, edgecolor=color, facecolor='none'
        )

    def inside_region(self, state) -> float | MX | DM:
        xmin = self.center[0] - self.width/2
        ymin = self.center[1] - self.height/2
        xmax = self.center[0] + self.width/2
        ymax = self.center[1] + self.height/2

        # b - A(x - xc) > 0
        return -logsumexp(-vertcat(
            state.x - xmin,
            xmax - state.x,
            state.y - ymin,
            ymax - state.y
        ), MARGIN)

    def outside_region(self, state) -> float | MX | DM:
        return -self.inside_region(state)

class Environment:

    def __init__(self):
        self.final : Region | list[Region] = None
        self.obstacles  : list[Region] = None

        self.po = PlotOptions(
            (-2.0, 25.0), (-2.0, 25.0), "Placeholder"
        )

    def set_final(self, final : Region | list[Region] = None):
        self.final = final
        if type(self.final) == list:
            self.multi_goal = True
            for fin in self.final:
                fin.set_patch_color("blue")
        else:
            self.multi_goal = False
            self.final.set_patch_color("blue")

    def set_obstacles(self, obstacles):
        self.obstacles = obstacles
        for obstacle in self.obstacles:
            obstacle.set_patch_color("red")

    def get_optimum_final_states(self) -> list[State]:
        optimum_states = []
        if self.multi_goal:
            for final in self.final:
                optimum_states.append(State(final.center[0], final.center[1], 0.0, 0.0))
        else:
            optimum_states.append(State(self.final.center[0], self.final.center[1], 0.0, 0.0))
        return optimum_states

    def get_environment(self):
        environment = []
        if self.final != None:
            if self.multi_goal:
                environment.extend(self.final)
            else:
                environment.append(self.final)
        
        if self.obstacles != None:
            environment.extend(self.obstacles)
        
        return environment

    def get_plot_options(self) -> PlotOptions:
        return self.po

def simple_environment_1() -> Environment:
    environ = Environment()
    environ.set_final(
        Rectangle(2.0, 2.0, (10.0, 10.0))
    )
    environ.set_obstacles(
        [
            Rectangle(2.0, 2.0, (5.0, 5.0))
        ]
    )
    return environ

def simple_environment_2() -> Environment:
    environ = Environment()
    environ.set_final(
        Circle(2.0, (10.0, 10.0))
    )
    environ.set_obstacles(
        [
            Circle(2.0, (5.0, 5.0))
        ]
    )
    return environ

def multi_simple_environment_1() -> Environment:
    environ = Environment()
    environ.set_final(
        [
            Rectangle(2.0, 2.0, (10.0, 10.0)),
            Rectangle(2.0, 2.0, (10.0, 20.0)),
            Rectangle(2.0, 2.0, (20.0, 20.0)),
            Rectangle(2.0, 2.0, (20.0, 10.0))
        ]
    )
    environ.set_obstacles(
        [
            Rectangle(6.0, 12.0, (15.0, 15.0)),
            Rectangle(12.0, 6.0, (15.0, 15.0))
        ]
    )
    environ.po.title = "Visit All"
    return environ

def multi_simple_environment_2() -> Environment:
    environ = Environment()
    environ.set_final(
        [
            Circle(1.0, (10.0, 10.0)),
            Circle(1.0, (20.0, 10.0)),
            Circle(1.0, (20.0, 20.0)),
            Circle(1.0, (10.0, 20.0))
        ]
    )
    environ.set_obstacles(
        [
            Rectangle(4.0, 1.0, (12.0, 20.0)),
            # Rectangle(1.0, 4.0, (10.0, 18.0)),
            # Rectangle(1.0, 4.0, (20.0, 12.0)),
            Rectangle(4.0, 1.0, (18.0, 10.0)),
            Rectangle(4.0, 1.0, (18.0, 20.0)),
            # Rectangle(1.0, 4.0, (20.0, 18.0)),
            Rectangle(4.0, 1.0, (12.0, 10.0)),
            # Rectangle(1.0, 4.0, (10.0, 12.0))
        ]
    )
    return environ

def narrow_environment() -> Environment:
    environ = Environment()
    environ.set_final(
        Rectangle(2.0, 2.0, (13.0, 7.0))
    )
    environ.set_obstacles(
        [
            Rectangle(12.0, 10.0, (0.0, 10.0)),
            Rectangle(12.0, 10.0, (12.0, 0.0)),
            Rectangle(2.0, 1.0, (10.0, 7.0)),
            Rectangle(8.0, 1.0, (10.0, 14.0)),
            Rectangle(10.0, 1.0, (4.0, -2.0))
        ]
    )
    environ.po.xlim = (-2, 15)
    environ.po.ylim = (-2, 12)
    environ.po.title = "Narrow Passage"
    return environ