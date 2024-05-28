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
ros2 run optimal_control dubins_vehicle.py
```

TermB:
```bash
./scripts/deploy/devel.sh
ros2 launch limo_simulation limo.launch.py
```