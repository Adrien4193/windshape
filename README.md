# Drone control in a wind test environment

Drone and WindShape fans array control using OptiTrack cameras with ROS.

## Documentation

Check doxygen documentation [here](https://adrien4193.github.io/windshape).

## Getting Started

### Prerequisites

This package has been tested on Ubuntu 16.04 in native and Virtual Machine with ROS Kinetic but other Linux and ROS configurations may work as well.

#### Hardware

To run this program, you will need:

1. A Windows computer connected to the OptiTrack cameras running Motive.

2. An Ubuntu computer (or Virtual Machine) connected on the same network as the Windows one.

3. A drone equipped of a PixHawk running PX4 auto-pilot.

4. A WiFi card or dongle on the Ubuntu computer to connect to the drone's WiFi access point.

5. A WindShape fans array connected on the same network as the Ubuntu computer.

#### Software

1. Install Ubuntu 16.04 ([download](http://releases.ubuntu.com/16.04)).

2. Install ROS Kinetic as described in [this tutorial](http://wiki.ros.org/kinetic/Installation/Ubuntu).

3. Configure your ROS environment as described [here](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

	Note: You will have to source your catkin workspace at each terminal prompt opening using:
	
	`source <path/to/your/catkin_ws>/devel/setup.bash`
	
	To avoid that, in a terminal prompt, you can do the following:
	
	Open bashrc (the script executed at each new terminal window) in gedit.
	
	`gedit ~/.bashrc`

	Then add this line at the end of the file (usually path is ~/catkin_ws):
	
	`source <path/to/your/catkin_ws>/devel/setup.bash`

4. Install vrpn_client_ros (to collect 6DOF data from Motive):

	`sudo apt-get install ros-kinetic-vrpn-client-ros`

5. Install MAVROS (to communicate with the flight controller running PX4 autopilot):
	
	Install the ROS package:
	
	`sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras`
	
	Then install GeographicLib datasets by running the install_geographiclib_datasets.sh script:

	```
	wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
	chmod +x install_geographiclib_datasets.sh
	sudo ./install_geographiclib_datasets.sh
	```

6. Numpy and MySQL Python API are necessary to run the main node. If not installed, it can be done using:

	```
	pip install numpy
	pip install MySQL-python
	```
	
	Note: If pip is not installed on your computer, use:
	
	`sudo apt-get install python-pip`

7. (Optional) Install QGroundControl to monitor your drone:

	First download the AppImage as described [here](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html).
	
	Then, if QGC crashes when you launch the AppImage, enter in a terminal prompt:
	
	```
	sudo usermod -a -G dialout $USER
	sudo apt-get remove modemanager
	```
	
	Note: QGC can also be installed on the Windows side.

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

You should see the interface and the drone will be automatically detected.

The parameters can be edited in windshape.yaml.

To see the replay of the plots:

`rosrun windshape replay`

You will have an interface to choose the log file (.txt).

## Video

Hover using offboard control: [video here](https://drive.google.com/open?id=1RmHsBMmI1-eMWGd4iuTmPswGTIf-fbPF).

Drone control in mode "follow target": [video here](https://drive.google.com/open?id=1739lTYdr3A3ERDlvbINxapeQnbph3eQg).

Drone control in mode "mimic target": [video here](https://drive.google.com/open?id=1CpWrYsjDy3I_B_7iK03V-LxNd--XeB8Q).

Drone control in mode "reach position": [video here](https://drive.google.com/open?id=1Rdp5rcwXBWxsB72oH-5NI5kMpFt8XfK4).

Wind control using drone pitch (better with sound): [video here](https://drive.google.com/open?id=1Drrw3u30u8EaXehTX62B3-S0HLGWBAgt).

Wind function: [video here](https://drive.google.com/open?id=1pshqk90JXQU0623GQq7QOQQnzgkKgJDV).

Drone and wind: [video here](https://drive.google.com/open?id=1b6yyCfbO6iSfz79UblGvRwTAiixKAUkF).

## Authors

* **Adrien Fleury** - *Package creation* - [Adrien4193](https://github.com/Adrien4193)

## License

This project is licensed under the BDS License - see the [LICENSE.md](LICENSE.md) file for details

## Bug reporting

Please report bugs (haha).
