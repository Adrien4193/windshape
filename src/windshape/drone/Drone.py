import weakref
import numpy

# ROS main library
import rospy

# Attitude control
from control.Controller import Controller
# Low-pass filters
from control.LowPassFilter import LowPassFilter
from control.MovingAverage import MovingAverage

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
	
	## Available tasks that the drone can perform (str)
	TASKS = ['reach_setpoint', 'follow_target']
	
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
		
		# Drone and target label
		drone = rospy.get_param('~tracking/tracker_drone')
		target = rospy.get_param('~tracking/tracker_target')
		
		# FCU
		self.__client = MAVROSClient()
		self.__listener = MAVROSListener()
		self.__publisher = MAVROSPublisher()
		
		# Mocap pose
		self.__rigidBody = RigidBody(drone)
		
		# Pose to reach
		self.__target = RigidBody(target)
		self.__setpoint = DronePose()
		self.__task = rospy.get_param('~control/task')
		self.__shift = numpy.array(rospy.get_param('~control/shift'))
		
		# Position control
		self.__controller = Controller()
		self.__filter = LowPassFilter(200, numpy.zeros(6))
		
		# Real time update
		self.__timers = []
		self.__initTimers()
		
		rospy.logdebug('Drone initialized')
		
	def __del__(self):
		"""Shutdown timers."""
		rospy.logdebug('Drone destruction')
		self.__close()
		
	# FCU FEEDBACK
	####################################################################
	
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
		
	# MOCAP TRACKING
	####################################################################
	
	def getMocapLabel(self):
		"""Returns the label of the drone's VRPN Tracker (str).
		
		Note:
			The tracker label is defined at rigid body creation on the
			mocap system.
		"""
		return self.__rigidBody.getLabel()
		
	def getMocapPose(self):
		"""Returns the drone pose from the mocap system (DronePose).
		
		Corresponds to the pose of the rigid body associated to the
		drone using setMocapLabel method.
		"""
		return self.__rigidBody.getPose()
		
	def isTracked(self):
		"""Returns True if the drone is tracked by the mocap system.
		
		Note:
			Based on heartbeats timeout from VRPN stream.
		"""
		return self.__rigidBody.isTracked()
		
	def setMocapLabel(self, label):
		"""Assigns a rigid body to the drone to measure its pose.
		
		Args:
			label (str): Label of the VRPN Tracker (from mocap system)
			
		Notes:
			getTrackersList returns the streamed VRPN Trackers.
		"""
		self.__rigidBody = RigidBody(label)
		
	# POSITION CONTROL
	####################################################################
	
	def getSetpoint(self):
		"""Returns the drone's current setpoint (DronePose).
		
		A setpoint is composed of X, Y, Z and Yaw coordinates only.
		
		The setpoint can either be the manual setpoint given with
		setManualSetpoint or the target pose given with setTarget.
		
		Use getTask to see what the drone is doing.
		
		Use setManualSetpoint, setTarget and setTask to change it.
		"""
		return self.__getSetpoint()
	
	def getManualSetpoint(self):
		"""Returns the last setpoint assigned manually (DronePose).
		
		A setpoint is composed of X, Y, Z and Yaw coordinates only.
		"""
		return self.__setpoint
		
	def getTarget(self):
		"""Returns the current target of the drone (RigidBody).
		
		See RigidBody to extract information about target.
		"""
		return self.__target
		
	def getTask(self):
		"""Returns the task the drone is performing.
		
		The task determines if the drone will try to follow a target or
		reach a manual setpoint.
		
		See TASKS for complete list.
		"""
		return self.__task
		
	def setManualSetpoint(self, x, y, z, yaw):
		"""Assigns a new setpoint to the drone.
		
		Args:
			x, y, z: Position to reach [m].
			yaw: Yaw to hover with [rad].
		
		Note:
			The setpoint will be reached only if drone is armed with its
			flight mode set to OFFBOARD and its task is to do so.
		"""
		self.__setpoint = DronePose(x, y, z, 0, 0, yaw)
		
	def setTarget(self, label):
		"""Assigns a target to the drone from its VRPN label (str).
		
		The target is a rigid body streamed in VRPN that the drone
		will have to follow with a position shift defined in config.
		
		Note:
			The setpoint will be reached only if drone is armed with its
			flight mode set to OFFBOARD and its task is to do so.
		"""
		self.__target = RigidBody(label)
		
	def setTask(self, task):
		"""Assigns a task to the drone.
		
		The task determines if the drone will try to follow a target or
		reach a manual setpoint.
		
		Args:
			task (str): The task to perform (see TASKS).
		"""
		self.__task = task
		
	# CONTROL PARAMETERS
	####################################################################
	
	def getControlParameters(self):
		"""Returns the parameters of the controller (Parameters).
		
		See Parameters to extract and change control attributes.
		"""
		return self.__controller.getParameters()
		
	# DRONE AS STR
	####################################################################
	
	def __str__(self):
		"""Returns a summary (str) of the drone status."""
		control = self.getControlParameters()
		
		fcu = '\n    '.join([
				'FCU:',
				'Connected: {}'.format(self.isConnected()),
				'Armed: {}'.format(self.isArmed()),
				'Mode: {}'.format(self.getFlightMode())])
			
		mocap = '\n    '.join([
				'Mocap:',
				'Stream: {}'.format(self.getTrackersList()),
				'Drone: {}'.format(self.getMocapLabel()),
				'Tracked: {}'.format(self.isTracked()),
				'Target: {}'.format(self.getTarget().getLabel()),
				'Tracked: {}'.format(self.getTarget().isTracked())])
		
		ctrl = '\n    '.join([
				'Control:',
				'Task: {}'.format(self.getTask()),
				'Mode: {}'.format(control.getMode()),
				'Mask: {}'.format(control.getMask())])
		
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
		self.__startTimer(rate, lambda t: drone().__sendMocap())
		
		# Sends position or attitude setpoint at defined rate
		rate = rospy.get_param('~update/control_rate')
		self.__startTimer(rate, lambda t: drone().__sendSetpoint())
		
		# Safety
		rospy.on_shutdown(lambda: drone().__close())
	
	def __startTimer(self, rate, callback):
		"""Starts a ROS timer to call callback at rate."""
		period = 1.0 / rate
		timer = rospy.Timer(rospy.Duration(period), callback)
		self.__timers.append(timer)
		
	def __close(self):
		"""Closes all timers."""
		for timer in self.__timers:
			timer.shutdown()
		self.__timers = []
		
	# PRIVATE COMPUTATION
	####################################################################
	
	def __computeAttitude(self):
		"""Returns RPYT to perform current task."""
		setpoint = self.getSetpoint().toArray()
		pose = self.getMocapPose().toArray()
		
		# Filters manual setpoint to avoid jerks
		if self.__task == 'reach_setpoint':
			setpoint = self.__filter(setpoint)
		
		# Calls position controller
		attitude = self.__controller(setpoint, pose)
		
		return DroneAttitude(*attitude)
		
	def __getSetpoint(self):
		"""Returns the setpoint (DronePose) to reach."""
		
		# Manual setpoint
		if self.__task == 'reach_setpoint':
			return self.__setpoint
		
		# Target pose shifted to avoid killing user
		elif self.__task == 'follow_target':
			setpoint = self.__target.getPose().toArray()
			setpoint += self.__shift
			return DronePose(*setpoint)
		
	def __sendMocap(self):
		"""Sends pose from mocap system to the drone."""
		pose = self.__rigidBody.getPoseStamped()
		self.__publisher.sendMocapPose(pose)
	
	def __sendSetpoint(self):
		"""Performs position control."""
		mode = self.getControlParameters().getMode()
		publisher = self.__publisher
		
		# Control is done offboard -> Sends attitude to reach setpoint
		if mode == "offboard":
			
			if not self.isArmed() or self.getFlightMode() != 'OFFBOARD':
				self.__controller.reset()
				attitude = DroneAttitude()
			else:
				attitude = self.__computeAttitude()
			
			publisher.sendSetpointAttitude(attitude.toPoseStamped())
			publisher.sendSetpointThrust(attitude.getThrust())
		
		# Control is done onboard -> Just sends pose to reach
		elif mode == 'onboard':
			self.__controller.reset()
			setpoint = self.getSetpoint().toPoseStamped()
			publisher.sendSetpointPosition(setpoint)
		
		# Safety
		else:
			rospy.logerr('Wrong control mode: %s', mode)
