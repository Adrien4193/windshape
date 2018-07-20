"""
Drone commander module.

Used to create a Drone instance (one at a time) and manage the real time
update of this one.

Call commander.create() method to create a Drone instance. This will
launch ROS master as subprocess and start ROS timers to update the
drone state, control it by sending its mocap pose and attitude setpoints
to the FCU and log information and measurements.

Usage:

	from windshape import commander
	
	drone = commander.create() # create Drone instance
	
	# Do your stuff with the drone (see Drone class)
	
	# Optional to stop drone update and free reference
	commander.close() # Auto called at exit
	
See the commander.py file for Drone management details.
See the Drone class (in drone) for more details about drone control.
"""
