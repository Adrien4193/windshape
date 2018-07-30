# Drone control in a wind test environment

Drone and WindShape fans array control using OptiTrack cameras with ROS.

## Documentation

Check doxygen documentation [here](https://adrien4193.github.io/windshape/index.html).

## Getting Started

### Prerequisites

This package has been tested on Ubuntu 16.04 in native and Virtual Machine with ROS Kinetic but other Linux and ROS configurations may work as well.

#### Hardware

To run this program, you will need:

1. A Windows computer connected to the OptiTrack cameras running Motive.

2. An Ubuntu computer (or Virtual Machine) connected on the same network as the Windows one.

3. A drone equipped of a PixHawk running PX4 auto-pilot.

4. A WiFi card or dongle to connect to the drone's WiFi access point.

5. A WindShape fans array connected on the same network as the Ubuntu computer.

#### Software

1. Install Ubuntu 16.04 ([download](http://releases.ubuntu.com/16.04)).

2. Install ROS Kinetic as described in [this tutorial](http://wiki.ros.org/kinetic/Installation/Ubuntu).

3. Configure your ROS environment as described [here](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

	Note: You will have to source your catkin workspace at each terminal prompt opening using:
	
	`source <path/to/your/catkin_ws>/devel/setup.bash`.
	
	To avoid that, in a terminal prompt, you can do the following:
	
	Open bashrc (the script executed at each new terminal window) in gedit.
	
	`gedit ~/.bashrc`

	Then add this line at the end of the file (usually path is ~/catkin_ws):
	
	`source <path/to/your/catkin_ws>/devel/setup.bash`

4. Install vrpn_client_ros (to collect 6DOF data from Motive):

	`sudo apt-get install ros-kinetic-vrpn-client-node`

5. Install MAVROS (to communicate with the flight controller running PX4 autopilot):
	
	Install the ROS package:
	
	`sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras`
	
	Then install GeographicLib datasets by running the install_geographiclib_datasets.sh script:

	```
	wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
	chmod +x install_geographiclib_datasets.sh
	sudo ./install_geographiclib_datasets.sh
	```

6. (Optional) Install QGroundControl to monitor your drone:

	First download the AppImage as described [here](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html).
	
	Then, if QGC crashes when you launch the AppImage, enter in a terminal prompt:
	
	```
	sudo usermod -a -G dialout $USER
	sudo apt-get remove modemanager
	```

### Installing

The latest version of windshape can be installed as follows:

In a terminal prompt, go to your catkin workspace source directory (usually located under $HOME/catkin_ws/src):

`cd ~/catkin_ws/src`

Download the source files (or fork and copy it manually):

`git clone git://github.com/Adrien4193/windshape.git`

Return to catkin root directory (catkin_ws here):

`cd ..`

Setup your environment using catkin (DO NOT run setup.py directly):

`catkin_make`

Setup your ROS environment to include the new development workspace:

`. devel/setup.bash`

## Running the tests

Run the GUI using roslaunch command:

`roslaunch windshape control.launch`

You should see the interface.

The parameters can be edited in windshape.yaml.

## Authors

* **Adrien Fleury** - *Package creation* - [Adrien4193](https://github.com/Adrien4193)

## License

This project is licensed under the BDS License - see the [LICENSE.md](LICENSE.md) file for details

## Bug reporting

Please report bugs (haha).
