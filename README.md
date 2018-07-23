WINDSHAPE
======================================================

Drone and WindShape fans array control using OptiTrack cameras.

Documentation
-------------

Check doxygen documentation [here](https://adrien4193.github.io/windshape/)

Getting windshape
-------------

If you just want to browse the repository, you can do so by going [here](https://github.com/Adrien4193/windshape).

The latest version of windshape can be installed as follows:

Go to your catkin workspace (usually located here):

`cd ~/catkin_ws/src`

Download the source files:

`git clone git://github.com/Adrien4193/windshape.git`

Return to the catkin packages directory (catkin_ws/src):

`cd ..`

Call the setup.py using catkin build tools:

`catkin_make`

Setup your ROS environment to include new development space:

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
