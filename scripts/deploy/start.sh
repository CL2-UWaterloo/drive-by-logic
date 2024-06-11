#!/bin/bash

# Get base.sh funcs
source "$(dirname "$0")/base.sh"

stop_docker

# declare mode, use gpu by default
mode="gpu"

# declare sim, use sim by default
sim="True"

while getopts 'csh' opt; do
    case "$opt" in
        c)
            mode="cpu"
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
        run_docker --runtime=nvidia optimal_control:sim "/root/app.sh -s"
    else
        run_docker --runtime=nvidia optimal_control:robot /root/app.sh
    fi
else
    if [ "$sim" == "True" ]; then
        run_docker optimal_control:sim "/root/app.sh -s"
    else
        run_docker optimal_control:robot /root/app.sh
    fi
fi