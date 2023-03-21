#!/bin/sh
docker build --compress --force-rm --build-arg CUR_UID=$(id -u) -t ros2-humble-jammy:latest -f Dockerfile.ros2-humble-jammy .
