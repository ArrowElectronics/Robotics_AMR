#!/bin/sh
docker stop ros2-humble-jammy &> /dev/null
docker rm -f ros2-humble-jammy &> /dev/null
mkdir -p adi-dev
docker run \
	--privileged \
	-itd \
	--init \
	--rm \
	--network host \
	--hostname humble \
	-v /dev:/dev \
	-v /tmp/.x11-unix:/tmp/.x11-unix \
	-e DISPLAY \
	-v $(pwd)/adi-dev:/home/analog/adi-dev:rw \
	-u $(id -u) \
	--name ros2-humble-jammy \
	ros2-humble-jammy:release
