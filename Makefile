link-ros:
	bash tools/ros/link_ros_packages.sh

build-wheel:
	bash tools/auto_build.sh

.PHONY: link-ros build-wheel