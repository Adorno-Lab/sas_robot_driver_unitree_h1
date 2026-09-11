#!/bin/bash

xhost +local:root
docker compose -f docker/simulator/Compose.yml up