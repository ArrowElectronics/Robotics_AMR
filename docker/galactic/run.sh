#!/bin/sh
docker stop ros2-galactic-focal &> /dev/null
docker rm -f ros2-galactic-focal &> /dev/null
mkdir -p adi-dev
docker run \
	--privileged \
	-itd \
	--init \
	--rm \
	--network host \
	-v /dev:/dev \
	-v /tmp/.x11-unix:/tmp/.x11-unix \
	-e DISPLAY \
	-v $(pwd)/adi-dev:/home/admin/adi-dev:rw \
	-u $(id -u) \
	--name ros2-galactic-focal \
	ros2-galactic-focal:release
