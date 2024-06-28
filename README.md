# Optrol

Optimization based control of non-holonomic systems

## Demonstration

[Video1](https://drive.google.com/file/d/1KJxkJGbfWq_1anegJha4giuoNKhBT3r2/view?usp=drive_link)

## Run Instructions

For setup:

```bash
./scripts/build/sim.sh
```

For deploy:

TermA:
```bash
./scripts/deploy/devel.sh
ros2 run optimal_control planner.py
```

TermB:
```bash
./scripts/deploy/devel.sh
ros2 launch limo_simulation limo.launch.py
```

## Timings (Raw)

Closed: mean, variance of time taken to traverse from (0, 0) to points ranging from (-10, -10) to (10, 10)

    feas: 0.07845346485260771 0.014564721790217018
    min_jerk: 0.3625517959183673 0.49373335906611254

Forward: mean, variance of time taken to traverse from (0, 0) to points ranging from (-10, -10) to (10, 10)

    feas: 0.09625590702947845 0.09453448430218862
    min_jerk: 0.4514233968253968 0.9096550141537358