WINDSHAPE
======================================================

Drone and WindShape fans array control using OptiTrack cameras.

Documentation
-------------

Check doxygen documentation [here](https://adrien4193.github.io/windshape/index.html).

Installation
-------------

This package has been tested using ROS Kinetic on Ubuntu 16.04.

See the tutorial to install ROS [here](http://wiki.ros.org/kinetic/Installation/Ubuntu).

Install vrpn_client_ros using:

`sudo apt-get install ros-kinetic-vrpn-client-node`

Install MAVROS using:

`sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras`

Then install GeographicLib datasets by running the install_geographiclib_datasets.sh script:

`wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh`

Getting windshape
-------------

If you just want to browse the repository, you can do so by going [here](https://github.com/Adrien4193/windshape).

The latest version of windshape can be installed as follows:

Go to your catkin workspace source directory (usually located under $HOME/catkin_ws/src):

`cd ~/catkin_ws/src`

Download the source files:

`git clone git://github.com/Adrien4193/windshape.git`

Return to the catkin packages directory (catkin_ws here):

`cd ..`

Setup your environment using catkin (DO NOT run setup.py directly):

`catkin_make`

Setup your ROS environment to include the new development workspace:

`. devel/setup.bash`

Using windshape
-------------

Run the GUI:

`roslaunch windshape control.launch`

You should see the interface.

Conditions of use
-----------------

windshape is distributed under the terms of the [BSD License](https://github.com/).

Bug reporting
-------------

Please report bugs (haha).
