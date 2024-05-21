#!/bin/bash

# Get base.sh funcs
source "$(dirname "$0")/base.sh"

stop_docker

# declare mode, use gpu by default
mode="gpu"

# declare sim, use sim by default
sim="True"

while getopts 'gsh' opt; do
    case "$opt" in
        g)
            mode="gpu"
            ;;
        s)
            sim="True"
            ;;
        ?|h)
            echo "Usage: $(basename $0) [-g] [-s]"
            exit 1
            ;;
    esac
done
shift "$(($OPTIND -1))"

if [ "$mode" == "gpu" ]; then
    if [ "$sim" == "True" ]; then
        run_docker --runtime=nvidia \
        -v $(dirname "$0")/../../workspace/:/root/workspace/src \
        optimal_control:sim bash
    else
        run_docker --runtime=nvidia \
        -v $(dirname "$0")/../../workspace/:/root/workspace/src \
        optimal_control:robot bash
    fi
else
    if [ "$sim" == "True" ]; then
        run_docker \
        -v $(dirname "$0")/../../workspace/:/root/workspace/src \
        optimal_control:sim bash
    else
        run_docker  \
        -v $(dirname "$0")/../../workspace/:/root/workspace/src \
        optimal_control:robot bash
    fi
fi