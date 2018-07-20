import math
import numpy

# ROS main library
import rospy

# Low-pass filter
from LowPassFilter import LowPassFilter
# Personal controller
from Controller import Controller

# Receives messages from mocap system
from ..mocap.RigidBody import RigidBody

# RPYT attitude with facilities
from ..common.DroneAttitude import DroneAttitude
# 6DOF pose with facilities
from ..common.DronePose import DronePose


class ControlManager(object):
	"""Manages the drone attitude control.
	
	Inherits from: object.
	
	Overrides: __init__, __del__.
	"""
	
	# CLASS ATTRIBUTES
	####################################################################
	
	TASKS = ['reach_setpoint', 'follow_target']
	CONTROL_MODES = ['onboard', 'offboard']
	
	# INITIALIZER AND DESTRUCTOR
	####################################################################
	
	def __init__(self):
		"""Initializes attributes."""
		rospy.logdebug('ControlManager initialization')
		
		# Target following
		tracker = rospy.get_param('~tracking/tracker_target')
		self.__shift = numpy.array(rospy.get_param('~control/shift'))
		
		# Parameters
		self.__task = rospy.get_param('~control/task')
		self.__mode = rospy.get_param('~control/mode')
		self.__mask = rospy.get_param('~control/mask')
		
		# Setpoints
		self.__setpoint = DronePose()
		self.__target = RigidBody(tracker)
		self.__manualInput = DroneAttitude()
		
		# Position controller
		self.__controller = Controller()
		
		# Command filter (for manual position control)
		self.__filter = LowPassFilter(200, numpy.zeros(6))
		
		rospy.logdebug('ControlManager initialized')
		
	def __del__(self):
		"""Just logs debug message."""
		rospy.logdebug('ControlManager destruction')
		
	# ATTITUDE COMPUTATION FOR POSITION CONTROL
	####################################################################
		
	def computeAttitude(self, dronePose):
		"""Returns RPYT to perform current task.
		
		Args:
			dronePose (DronePose): Measured pose of the drone
		"""
		if self.__mode != "offboard":
			rospy.logerr('Wrong mode: %s', self.__mode)
			return DroneAttitude()
		
		# Uses custom offboard position controller to perform task
		setpoint = self.getSetpoint().toArray()
		pose = dronePose.toArray()
		
		# Filters static setpoint to avoid jerks
		if self.__task == 'reach_setpoint':
			setpoint = self.__filter(setpoint)
		
		# Calls position controller
		attitude = self.__controller(setpoint, pose)
		
		return DroneAttitude(*attitude)
		
	# PARAMETERS
	####################################################################
	
	def getMask(self):
		"""Returns the current attitude mask (bool[4])."""
		return self.__mask
	
	def getMode(self):
		"""Returns the active control mode (str)."""
		return self.__mode
		
	def getTask(self):
		"""Returns the task the drone is performing (str)."""
		return self.__task
		
	def setMask(self, mask):
		"""Masks (bool[4]) attitude components set to True."""
		
		if len(mask) != 4:
			rospy.logerr('Wrong mask: %s', mask)
		
		self.__mask = mask
		self.__controller.setMask(mask)
		
	def setMode(self, mode):
		"""Set control mode (str)."""
		
		if mode not in ControlManager.CONTROL_MODES:
			rospy.logerr('unknown mode: %s', mode)
		
		self.__mode = mode
		
	def setTask(self, task):
		"""Assigns a task (str) to the drone."""
		
		if task not in ControlManager.TASKS:
			rospy.logerr('unknown task: %s', task)
		
		self.__task = task
		
	# POSITION CONTROL
	####################################################################
	
	def getSetpoint(self):
		"""Returns the setpoint to reach (DronePose)."""
		
		if self.__task == 'reach_setpoint':
			return self.__setpoint
			
		elif self.__task == 'follow_target':
			setpoint = self.__target.getPose().toArray()
			setpoint += self.__shift
			return DronePose(*setpoint)
			
		else:
			rospy.logerr('Unknown task: %s', task)
			return DronePose()
		
	def getTarget(self):
		"""Returns the current target (RigidBody)."""
		return self.__target
		
	def setSetpoint(self, x, y, z, yaw):
		"""Set new setpoint to reach and hover [m, rad]."""
		self.__setpoint = DronePose(x, y, z, 0, 0, yaw)
	
	def setTarget(self, label):
		"""Set target VRPN Tracker from its label (str)."""
		self.__target = RigidBody(label)
	
	# ATTITUDE CONTROL
	####################################################################
	
	def getControlInput(self):
		"""Returns the last attitude sent (DroneAttitude).
		
		Note:
			Used for display, does not compute anything.
		"""
		if self.__mode == 'onboard':
			return DroneAttitude()
			
		elif self.__mode == 'offboard':
			attitude = self.__controller.getControlInput()
			return DroneAttitude(*attitude)
			
		else:
			rospy.logerr('Wrong control mode: %s', self.__mode)
			return DroneAttitude()
			
	def getError(self):
		"""Returns the error between setpoint and pose."""
		return DroneAttitude(*self.__controller.getError())
			
	def setManualInput(self, roll, pitch, yaw, thrust):
		"""Sends attitude to the drone manually [rad, 0-1]."""
		self.__manualInput = DroneAttitude(roll, pitch, yaw, thrust)
		self.__controller.setManualInputs(self.__manualInput.toArray())
		
	def reset(self):
		"""Resets controller."""
		self.__controller.reset()
