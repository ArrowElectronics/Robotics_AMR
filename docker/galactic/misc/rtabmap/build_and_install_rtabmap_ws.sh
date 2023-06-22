#!/bin/bash
set -e

build_and_install_rtabmap_ws()
{
	pushd $1

	echo "Building and Installing rtabmap_ws at: $1"
	mkdir -p src
	vcs import --input ${BASEDIR}/rtabmap-ws.repos ./src
		
	if [[ -d ${BASEDIR}/patches/rtabmap/ ]]; then
		echo "Patching rtabmap"
		pushd src/rtabmap
		git config user.email "test@example.com"
		git config user.name "Your Name"
		git am ${BASEDIR}/patches/rtabmap/*.patch
		popd
	fi

	echo "Here: `pwd`"
	source /opt/ros/$ROS_DISTRO/setup.bash
	echo "${SUDO_PASSWD}" | sudo -ES rosdep install --from-paths ./src --ignore-packages-from-source -y
	colcon build --symlink-install --cmake-force-configure --parallel-workers ${PARALLEL_WORKERS}
	popd
}


print_usage()
{
	echo "Usage: $0 <INSTALL_PATH>"
	exit 1
}

main()
{
	SCRIPT_PATH=`readlink -f "${BASH_SOURCE:-$0}"`
	BASEDIR=$(dirname $SCRIPT_PATH)
	if [ "$#" -ne 1 ] || ! [ -d "$1" ]; then
		print_usage $0
	fi
	
	SUDO_PASSWD="${SUDO_PASSWD:-analog}"
	PARALLEL_WORKERS="${PARALLEL_WORKERS:-`nproc`}"
	echo "Installing to Path: $1 with parallel workers ${PARALLEL_WORKERS}"

	build_and_install_rtabmap_ws $1
}

main "$@"
