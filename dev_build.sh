#!/bin/bash

# Build the docker image but don't build the source code
docker build \
    --build-arg BUILD_CODE=false \
    -t sas_robot_driver_unitree_h1 \
    -f docker/Dockerfile \
    .