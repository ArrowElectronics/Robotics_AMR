#!/bin/sh
docker stop ros1-noetic-ros2-galactic > /dev/null
docker rm -f ros1-noetic-ros2-galactic > /dev/null
mkdir -p data
docker run \
	--privileged \
	-itd \
	--init \
	--rm \
	--network host \
	-v /dev:/dev \
	-v /tmp/.x11-unix:/tmp/.x11-unix \
	-e DISPLAY \
	-v $(pwd)/data:/home/admin/data:rw \
	-u $(id -u) \
	--name ros1-noetic-ros2-galactic \
	ros1-noetic-ros2-galactic:latest
