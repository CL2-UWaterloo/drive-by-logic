#!/bin/bash

docker build --rm $@ -t optimal_control:sim -f "$(dirname "$0")/../../docker/sim.Dockerfile" .