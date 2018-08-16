cmake_minimum_required(VERSION 2.8.3)

# Project name (package)
project(windshape)

# Dependencies
find_package(catkin REQUIRED COMPONENTS rospy)

# Tells catkin to use our setup.py
catkin_python_setup()

# Loads environment variables
catkin_package()

# Tells catkin which files are executables
catkin_install_python(PROGRAMS
	nodes/test
	nodes/gui
	scripts/replay
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
