#!/bin/sh
docker build --build-arg CUR_UID=$(id -u) -t ros1-noetic-ros2-galactic:latest -f Dockerfile.ros1-noetic-ros2-galactic .
