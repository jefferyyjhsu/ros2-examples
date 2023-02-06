#!/bin/bash
rm -r build/ install/
colcon build --merge-install --packages-select examples_rclcpp_minimal_subscriber --cmake-force-configure --cmake-args -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON -DTHIRDPARTY=ON -DBUILD_TESTING:BOOL=OFF
