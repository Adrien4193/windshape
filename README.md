WINDSHAPE
======================================================

Drone and WindShape fans array control using OptiTrack cameras.

Documentation
-------------

Check doxygen documentation [here](https://adrien4193.github.io/windshape/index.html).

Installation
-------------

This package has been tested using ROS Kinetic on Ubuntu 16.04.

If not done, install ROS and configure your environment using [this tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

Install vrpn_client_ros using:

`sudo apt-get install ros-kinetic-vrpn-client-node`

Install MAVROS using:

`sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras`

Then install GeographicLib datasets by running the install_geographiclib_datasets.sh script (you might need to give him authorizations and use sudo as shown below):

`wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh`

You might also want to use QGroundControl to connect to the drone flight controller and change some parameters.

First download the App image as described [here](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html).

Note: If QGC crashes when you launch the App image, enter in a terminal prompt:

`sudo usermod -a -G dialout $USER
sudo apt-get remove modemanager`

Getting windshape
-------------

If you just want to browse the repository, you can do so by going [here](https://github.com/Adrien4193/windshape).

The latest version of windshape can be installed as follows:

Go to your catkin workspace source directory (usually located under $HOME/catkin_ws/src):

`cd ~/catkin_ws/src`

Download the source files (or fork and copy it manually):

`git clone git://github.com/Adrien4193/windshape.git`

Return to the catkin packages directory (catkin_ws here):

`cd ..`

Setup your environment using catkin (DO NOT run setup.py directly):

`catkin_make`

Setup your ROS environment to include the new development workspace:

`. devel/setup.bash`

Note: Don't forget to source your catkin workspace at each session using the setup.bash file located in catkin_ws/devel:

`source <path/to/your/catkin_ws>/devel/setup.bash`

Using windshape
-------------

Run the GUI using roslaunch command:

`roslaunch windshape control.launch`

You should see the interface.

Conditions of use
-----------------

windshape is distributed under the terms of the [BSD License](https://github.com/).

Bug reporting
-------------

Please report bugs (haha).
