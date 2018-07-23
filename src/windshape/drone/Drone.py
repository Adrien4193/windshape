import weakref

# ROS main library
import rospy

# Attitude control
from control.Controller import Controller

# Receives messages from the drone
from fcu.MAVROSListener import MAVROSListener
# Sends commands to the drone
from fcu.MAVROSClient import MAVROSClient
# Sends messages to the drone
from fcu.MAVROSPublisher import MAVROSPublisher

# Receives messages from mocap system
from mocap.RigidBody import RigidBody

# RPYT attitude with facilities
from common.DroneAttitude import DroneAttitude
# 6DOF pose with facilities
from common.DronePose import DronePose


class Drone(object):
	"""Header class to control the drone.
	
	The control is performed in separated threads using ROS Timers.
	
	Control parameters:
		Setpoint: Static pose (x, y, z, yaw) to reach.
		Target: Rigid body to follow (mocap label) with given shift.
		Task: Choose between the two tasks above (see TASKS).
	
	Notes:
		Units are always [m], [rad], [0-1].
		The global frame coordinate used is the one of the mocap system.
		The drone must be aligned with the X axis of the global frame
		BEFORE creating its rigid body.
	
	Inherits from: object.
	
	Overrides: __init__, __del__, __str__.
	"""
	
	# CLASS ATTRIBUTES
	####################################################################
	
	## Available flight modes for the drone (str)
	FLIGHT_MODES = MAVROSClient.PX4_FLIGHT_MODES
	
	# STATIC METHODS
	####################################################################
	
	@staticmethod
	def getTrackersList():
		"""Returns the list of availables VRPN Trackers (str list).
		
		Based on the vrpn_client_ros published topics names. Should
		correspond to the rigid bodies list of the mocap system.
		
		Note:
			Only trackers that have sent their pose at least once will
			be in the list.
		"""
		return RigidBody.getTrackersList()
		
	# INITIALIZER AND DESTRUCTOR
	####################################################################
	
	def __init__(self):
		"""Creates all children instances and start RT update."""
		rospy.logdebug('Drone initialization')
		
		# FCU
		self.__client = MAVROSClient()
		self.__listener = MAVROSListener()
		self.__publisher = MAVROSPublisher()
		
		# Mocap
		self.__body = RigidBody(rospy.get_param('~tracking/tracker'))
		
		# Position control
		self.__controller = Controller()
		
		# Real time update
		self.__timers = []
		self.__initTimers()
		
		rospy.logdebug('Drone initialized')
		
	def __del__(self):
		"""Shutdown timers."""
		rospy.logdebug('Drone destruction')
		Drone.__close(weakref.ref(self))
		
	# ATTRIBUTES GETTERS
	####################################################################
	
	def getControlParameters(self):
		"""Returns the parameters of the controller (Parameters).
		
		See Parameters to extract and change control attributes.
		"""
		return self.__controller.getParameters()
	
	def getEstimatedPose(self):
		"""Returns the drone pose estimated by the FCU (DronePose).
		
		Note:
			Estimation depends on parameters set on QGoundControl.
		"""
		return DronePose.fromPoseStamped(self.__listener.getPose())
		
	def getFlightMode(self):
		"""Returns the drone's active flight mode (str).
		
		See MAVROSClient for available flight modes list.
		
		Uses topic /mavros/state to get the drone state.
		"""
		return self.__listener.getState().mode
		
	def getMocapLabel(self):
		"""Returns the label of the drone's VRPN Tracker (str).
		
		Note:
			The tracker label is defined at rigid body creation on the
			mocap system.
		"""
		return self.__body.getLabel()
		
	def getMocapPose(self):
		"""Returns the drone pose from the mocap system (DronePose).
		
		Corresponds to the pose of the rigid body associated to the
		drone using setMocapLabel method.
		"""
		return self.__body.getPose()
		
	def isArmed(self):
		"""Returns True if the drone is armed.
		
		Uses topic /mavros/state to get the drone state.
		"""
		return self.__listener.getState().armed
		
	def isConnected(self):
		"""Returns True if the drone is connected.
		
		Uses topic /mavros/state to get the drone state.
		"""
		return self.__listener.getState().connected
		
	def isTracked(self):
		"""Returns True if the drone is tracked by the mocap system.
		
		Note:
			Based on heartbeats timeout from VRPN stream.
		"""
		return self.__body.isTracked()
		
	# ATTRIBUTES SETTERS
	####################################################################
	
	def setMocapLabel(self, label):
		"""Assigns a rigid body to the drone to measure its pose.
		
		Args:
			label (str): Label of the VRPN Tracker (from mocap system)
			
		Notes:
			getTrackersList returns the streamed VRPN Trackers.
		"""
		self.__body = RigidBody(label)
		
	# COMMANDS
	####################################################################
	
	def arm(self, cmd):
		"""Arms or disarms the drone.
		
		Uses MAVROS service "/mavros/cmd/arming" to send MAVLink arming
		message and get the response message (success or not).
		
		Args:
			cmd (bool): Arm if True, else disarm.
		"""
		self.__client.callArming(cmd)
		
	def land(self):
		"""Lands the drone using PX4 auto-land.
		
		Uses MAVROS service "/mavros/cmd/land" to send MAVLink landing
		message and get the response message (success or not).
		"""
		self.__client.callLand()
		
	def setFlightMode(self, mode):
		"""Sets drone active flight mode.
		
		Uses MAVROS service "/mavros/set_mode" to send MAVLink setMode
		message and get the response message (success or not).
		
		Args:
			mode (str): Mode to set (see getFlightModes).
		"""
		self.__client.callSetMode(mode)
		
	def takeoff(self):
		"""Takeoffs the drone using PX4 auto-takeoff.
		
		Uses MAVROS service "/mavros/cmd/takeoff" to send MAVLink
		takeoff message and get the response message (success or not).
		"""
		self.__client.callTakeoff()
		
	# SPECIAL METHODS
	####################################################################
	
	def __str__(self):
		"""Returns a summary (str) of the drone status."""
		control = self.getControlParameters()
		
		fcu = '\n    '.join([
				'FCU:',
				'Connected: {}'.format(self.isConnected()),
				'Armed: {}'.format(self.isArmed()),
				'Mode: {}'.format(self.getFlightMode())
				])
			
		mocap = '\n    '.join([
				'Mocap:',
				'Stream: {}'.format(self.getTrackersList()),
				'Label: {}'.format(self.getMocapLabel()),
				'Tracked: {}'.format(self.isTracked())
				])
		
		ctrl = '\n    '.join([
				'Control:',
				'Target: {}'.format(control.getTarget().getLabel()),
				'Tracked: {}'.format(control.getTarget().isTracked()),
				'Follow: {}'.format(control.isFollowingTarget()),
				'Mimic: {}'.format(control.isMimingTarget()),
				'Offboard: {}'.format(control.isUsingOffboardControl()),
				'Mask: {}'.format(control.getMask())
				])
		
		return '\n\n'.join([fcu, mocap, ctrl])
		
	# REAL-TIME UPDATE
	####################################################################
	
	def __initTimers(self):
		"""Starts ROS timers."""
		
		# IMPORTANT: (will block destructor otherwise)
		# Create weakref on instance
		drone = weakref.ref(self)
		
		# Sends drone's mocap pose at defined rate	
		rate = rospy.get_param('~update/mocap_sending_rate')
		self.__startTimer(rate, lambda t: Drone.__sendMocap(drone))
		
		# Sends position or attitude setpoint at defined rate
		rate = rospy.get_param('~update/control_rate')
		self.__startTimer(rate, lambda t: Drone.__sendSetpoint(drone))
		
		# Safety
		rospy.on_shutdown(lambda: Drone.__close(drone))
	
	def __startTimer(self, rate, callback):
		"""Starts a ROS timer to call callback at rate."""
		period = 1.0 / rate
		timer = rospy.Timer(rospy.Duration(period), callback)
		self.__timers.append(timer)
		
	@staticmethod
	def __close(drone):
		"""Closes all timers of drone (weakref)."""
		if drone() is not None:
			for timer in drone().__timers:
				timer.shutdown()
			drone().__timers = []
	
	@staticmethod
	def __sendMocap(drone):
		"""Sends pose from mocap system to the drone (weakref)."""
		if drone() is not None:
			pose = drone().__body.getPoseStamped()
			drone().__publisher.sendMocapPose(pose)
		
	@staticmethod
	def __sendSetpoint(drone):
		"""Performs position control on drone (weakref)."""
		if drone() is None:
			return
		
		control = drone().getControlParameters()
		pose = drone().getMocapPose().toArray()
		estimate = drone().getEstimatedPose().toArray()
		flightMode = drone().getFlightMode()
		publisher = drone().__publisher
		
		# Control is done offboard -> Sends attitude to reach setpoint
		if control.isUsingOffboardControl():
			
			if not drone().isArmed() or flightMode != 'OFFBOARD':
				drone().__controller.reset()
				attitude = DroneAttitude()
			else:
				controlInput = drone().__controller(pose, estimate)
				attitude = DroneAttitude(*controlInput)
			
			publisher.sendSetpointAttitude(attitude.toPoseStamped())
			publisher.sendSetpointThrust(attitude.getThrust())
		
		# Control is done onboard -> Just sends pose to reach
		else:
			drone().__controller.reset()
			setpoint = control.getSetpoint().toPoseStamped()
			publisher.sendSetpointPosition(setpoint)
