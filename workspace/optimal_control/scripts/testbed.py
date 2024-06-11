#!/usr/bin/python3

import matplotlib.pyplot as plt

from optimal_control.car_planner import *
from optimal_control.executor import Executor

if __name__ == "__main__":
    # map = [
    #     [5.0, 5.0, 1.0, 1.0],
    #     [-5.0, 5.0, 1.0, 1.0],
    #     [5.0, -5.0, 1.0, 1.0],
    #     [-5.0, -5.0, 1.0, 1.0]
    # ]

    lm = LimoBot()
    dc = CarPlanner(lm, granularity=10)

    ex = Executor(dc)
    init = State(0.0, 0.0, 0.0)

    x = []; y = []
    file_string = ""

    text_file = open("output.txt", "w")
    for i in range(-10, 11, 1):
        for j in range(-10, 11, 1):
            if i == 0 and j == 0:
                continue
            
            final = State(float(i), float(j), 0.0)
            ex.prep(init=init, final=final, obstacles=[])
            solution, solver = ex.solve()

            if (solver.stats()["success"]):
                file_string = str(i) + ", " + str(j) + ", " + str(solver.stats()["t_proc_total"]) + "\n"
                x.append(i); y.append(j)
            else:
                file_string = str(i) + ", " + str(j) + ", " + "0.0" + "\n"

            text_file.write(file_string)

    fig, ax = plt.subplots()
    ax.plot(x, y, "ro")
    plt.show()

    text_file.close()