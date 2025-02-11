#!/usr/bin/python3

import subprocess as sp
import re
import statistics

regex = "[,]?[+-]?\d+\.?\d*"

if __name__ == "__main__":
    experiment_name = "overtake"

    # Last column to indicate the number of cases of success or failure

    total = 10
    opt_success = 0; opt_times = []; opt_robust = []
    start_points = []
    for i in range(total):
        result = sp.run(
            ["ros2 run optimal_control "+experiment_name+".py"],
            capture_output=True,
            shell=True, text=True
        )

        to_be_parsed = result.stdout.splitlines()
 
        opt_pass = False
        for line in to_be_parsed:
            if "perturb x" in line:
                perturb_x = float(re.findall(regex, line)[0])

            if "perturb y" in line:
                perturb_y = float(re.findall(regex, line)[0])

        if (perturb_x, perturb_y) in start_points:
            opt_success += 1
            continue
        else:
            start_points.append((perturb_x, perturb_y))

        for line in to_be_parsed:
            if "Total seconds in IPOPT" in line:
                opt_time = float(re.findall(regex, line)[0])

            if "Optimization Conclusion" in line:
                if "True" in line:
                    opt_success += 1
                    opt_pass = True
                    opt_times.append(opt_time)
                    continue

            if "Robustness Value" in line:
                opt_rob = float(re.findall(regex, line)[0])

            if opt_pass:
                opt_robust.append(opt_rob)

        print(opt_pass, opt_success, i+1, opt_time, opt_rob, perturb_x, perturb_y)

    print("Success Rate: ", opt_success/total)
    print("Mean Runtime: ", statistics.mean(opt_times), statistics.stdev(opt_times))
    print("Mean Robustness: ", statistics.mean(opt_robust), statistics.stdev(opt_robust))