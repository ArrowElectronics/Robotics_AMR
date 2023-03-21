#!/bin/bash
set -e

build_and_install_ros1_noetic()
{
	pushd $1

	mkdir -p noetic_ws/src
	pushd noetic_ws
	vcs import --input ${BASEDIR}/noetic-ros_core.rosinstall ./src
	ROS_DISTRO= echo "${SUDO_PASSWD}" | sudo -ES rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y
	ROS_DISTRO= ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --make-args "-j${PARALLEL_WORKERS}"

	popd
}

build_and_install_ros1_bridge()
{
	pushd $1

	ROS1_INSTALL_PATH=$2
	ROS2_INSTALL_PATH=$3
	echo "Building ros1 bridge for ${ROS1_INSTALL_PATH} ---> ${ROS2_INSTALL_PATH}"
	mkdir -p ros1_bridge_ws/src
	pushd ros1_bridge_ws
	git clone https://github.com/ros2/ros1_bridge.git src/ros1_bridge
	source ${ROS1_INSTALL_PATH}/setup.bash
	source ${ROS2_INSTALL_PATH}/setup.bash
	colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure --parallel-workers ${PARALLEL_WORKERS}
	popd
}


print_usage()
{
	echo "Usage: $0 <INSTALL_PATH>"
	exit 1
}

main()
{
	BASEDIR=$(dirname $0)
	if [ "$#" -ne 1 ] || ! [ -d "$1" ]; then
		print_usage $0
	fi
	
	SUDO_PASSWD="${SUDO_PASSWD:-analog}"
	PARALLEL_WORKERS="${PARALLEL_WORKERS:-`nproc`}"
	echo "Installing to Path: $1 with parallel workers ${PARALLEL_WORKERS}"

	build_and_install_ros1_noetic $1
	build_and_install_ros1_bridge $1 $1/noetic_ws/install_isolated "${ROS2_PATH:-/opt/ros/humble}"
}

main "$@"
