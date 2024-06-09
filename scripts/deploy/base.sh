#!/bin/bash


run_docker() {
    xhost +local:root # Giving display privileges

    # -it is for interactive, tty
    # --privileged for accessing /dev contents
    # --net=host to share the same network as host machine. TL;DR same IP.
    docker run -it --privileged --net=host \
    --name optimal_control \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    -v $(dirname "$0")/app.sh:/root/app.sh \
    $@
}

stop_docker() {
    docker stop optimal_control && docker rm optimal_control
    xhost -local:root # Remove display privileges
}