#!/usr/bin/python3

from optimal_control.environment import *
from optimal_control.plotter import *

if __name__ == "__main__":
    plotter = Plotter(highway().get_plot_options(), None, highway().get_environment())
    plotter.show_plot()