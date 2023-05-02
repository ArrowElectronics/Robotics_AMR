#!/bin/sh
cp -r ../../eic_adi_sensor_fusion/ .
cp -r ../../eic_bringup_utils/ .
cp -r ../../imu_driver/ .
docker build --build-arg CUR_UID=$(id -u) -t ros2-galactic-focal:latest -f Dockerfile.ros1-noetic-ros2-galactic .
rm -rf eic_adi_sensor_fusion/ eic_bringup_utils/ imu_driver/
