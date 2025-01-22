#!/usr/bin/python3

from optimal_control.environment import *
from optimal_control.plotter import *

if __name__ == "__main__":
    plotter = Plotter(narrow_environment().get_plot_options(), None, narrow_environment().get_environment())
    plotter.show_plot()