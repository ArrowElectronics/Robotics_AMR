#!/bin/sh
cp -r ../../eic_adi_sensor_fusion/ .
cp -r ../../eic_bringup_utils/ .
cp -r ../../imu_driver/ .
cp -r ../../scripts/ .
docker build --compress --force-rm --build-arg CUR_UID=$(id -u) -t ros2-humble-jammy:release -f Dockerfile.ros2-humble-jammy .
rm -rf eic_adi_sensor_fusion/ eic_bringup_utils/ imu_driver/ scripts/
