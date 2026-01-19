link-ros:
	bash tools/ros/link_ros_packages.sh

build-wheel:
	bash tools/build_wheel.sh

.PHONY: link-ros build-wheel