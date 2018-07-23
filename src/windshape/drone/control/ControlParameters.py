import numpy

# ROS main library
import rospy

# RPYT attitude with facilities
from ..common.DroneAttitude import DroneAttitude
# 6DOF pose with facilities
from ..common.DronePose import DronePose


class ControlParameters(object):
	"""Control parameters used in offboard.
	
	Inherits from: object.
	
	Overrides: __init__, __del__.
	"""
	
	# INITIALIZER AND DESTRUCTOR
	####################################################################
	
	def __init__(self):
		"""Loads parameters."""
		
		# Target
		self.__target = RigidBody(rospy.get_param('~control/target'))
		self.__follow = rospy.get_param('~control/follow_target')
		self.__shift = numpy.array(rospy.get_param('~control/shift'))
		self.__mimic = rospy.get_param('~control/use_target_attitude')
		
		# Manual pose and attitude to reach
		self.__setpoint = DronePose()
		self.__attitude = DroneAttitude()
		
		# Use or not offboard position controller
		self.__offboard = rospy.get_param('~control/use_ws_controller')
		self.__mask = rospy.get_param('~control/mask')
		
		# Display
		self.__controlInput = DroneAttitude()
		self.__error = DroneAttitude()
		self.__separatedOutputs = 3*[DroneAttitude()]
		
	def __del__(self):
		"""Does nothing special."""
		pass
	
	# PARAMETERS GETTERS
	####################################################################
	
	def getAttitude(self):
		"""Returns attitude replacement to control (DroneAttitude).
		
		The attitude is composed of roll, pitch, yaw, thrust.
		
		The attitude can either be the manual attitude given with
		setManualAttitude or the target attitude given with setTarget.
		"""
		return self.__getAttitude()
		
	def getManualAttitude(self):
		"""Returns the last attitude entered by user (DroneAttitude).
		
		Replaces controller output if mask set as True.
		"""
		return self.__attitude
		
	def getManualSetpoint(self):
		"""Returns the last setpoint assigned manually (DronePose).
		
		A setpoint is composed of X, Y, Z and Yaw coordinates only.
		"""
		return self.__setpoint
		
	def getMask(self):
		"""Returns the control mask (bool[4]).
		
		Order: roll, pitch, yaw, thrust
		
		True = Replace controller output field by manual attitude.
		False = Use controller output.
		"""
		return self.__mask
		
	def getSetpoint(self):
		"""Returns the drone's current setpoint (DronePose).
		
		A setpoint is composed of X, Y, Z and Yaw coordinates only.
		
		The setpoint can either be the manual setpoint given with
		setManualSetpoint or the target pose given with setTarget.
		"""
		return self.__getSetpoint()
		
	def getTarget(self):
		"""Returns the current target of the drone (RigidBody).
		
		See RigidBody to extract information about target.
		"""
		return self.__target
		
	def isFollowingTarget(self):
		"""Returns True if the drone is following its target.
		
		If False, the drone will try to reach a static setpoint.
		"""
		return self.__follow
		
	def isUsingTargetAttitude(self):
		"""Returns True if attitude control based on the target one.
		
		If False, the drone will try to reach a static attitude.
		
		Notes:
			Will only have an effect on masked attitude fields.
		"""
		return self.__mimic
		
	def isUsingWSController(self):
		"""Returns True if the controller is used for position control.
		
		If False, onboard PX4 controller will be used.
		"""
		return self.__offboard
		
	# PARAMETERS SETTERS
	####################################################################
	
	def followTarget(self, follow):
		"""Defines if the drone is following its target.
		
		Args:
			follow (bool): Follow target if True.
		"""
		self.__follow = follow
		
	def setManualAttitude(self, roll, pitch, yaw, thrust):
		"""Assigns manually an attitude to the drone.
		
		Args:
			roll, pitch, yaw: Orientation of the drone [rad]
			thrust: Thrust of the drone [0-1]
			
		Note:
			- Disabled if not using WS controller.
			- Needs to be armed with OFFBOARD flight mode.
			- Will have an effect only if the corresponding component is
				set to True in the mask.
		"""
		self.__attitude = DroneAttitude(roll, pitch, yaw, thrust)
		
	def setManualSetpoint(self, x, y, z, yaw):
		"""Assigns a new setpoint to the drone.
		
		Args:
			x, y, z: Position to reach [m].
			yaw: Yaw to hover with [rad].
		
		Note:
			- Needs to be armed with OFFBOARD flight mode.
			- Will be reached only if not following target.
		"""
		self.__setpoint = DronePose(x, y, z, 0, 0, yaw)
		
	def setMask(self, roll, pitch, yaw, thrust):
		"""Enable or disable component of the drone attitude control.
		
		True = Replace controller output field by custom attitude.
		False = Use controller output.
		
		Args:
			roll, pitch, yaw, thrust (bool): True to replace component.
		"""
		self.__mask = [roll, pitch, yaw, thrust]
		
	def setTarget(self, label):
		"""Assigns a target to the drone from its VRPN label (str).
		
		The target is a rigid body streamed in VRPN that the drone
		will have to follow with a position shift defined in config.
		
		Note:
			- Needs to be armed with OFFBOARD flight mode.
			- Will be reached only if following target.
			- Will be imitated only if is using target attitude.
		"""
		self.__target = RigidBody(label)
		
	def useTargetAttitude(self, use):
		"""Defines the way the attitude control is done.
		
		Uses manual attitude if False, else target one (RPYZ).
		
		Args:
			use (bool): Use target attitude if True.
		"""
		self.__mimic = use
		
	def useWSController(self, use):
		"""Defines if the position control is done offboard.
		
		Uses PX4 onboard controller if False.
		
		Args:
			use (bool): Use offboard position control if True.
		"""
		self.__offboard = use
		
	# CONTROL INPUTS RECORDS GETTERS
	####################################################################
	
	def getControlInput(self):
		"""Returns the last control input sent (DroneAttitude).
		
		The control input is an attitude (roll, pitch, yaw, thrust).
		
		Notes:
			- Is zero if no offboard control is performed.
			- Does not compute anything, just displays.
		"""
		return self.__controlInput
		
	def getError(self):
		"""Returns the last error (setpoint - pose) (DroneAttitude).
		
		The error is composed of X, Y, Yaw, Z (X and Y are
		projected on the drone body frame).
		"""
		return self.__error
		
	def getSeparatedOutputs(self):
		"""Returns separated values of the PIDs (DroneAttitude[3]).
		
		0 = P-term, 1 = I-term, 2 = D-term.
		"""
		return self.__separatedOutputs
		
	# PROTECTED CONTROLLER SETTERS
	####################################################################
	
	def _setControlInput(self, attitude):
		"""Fills control input field (numpy.array[4])."""
		self.__controlInput = DroneAttitude(*attitude)
		
	def _setError(self, error):
		"""Fills error field (numpy.array[4])."""
		self.__error = DroneAttitude(*error)
		
	def _setSetparatedOutputs(self, p, i, d):
		"""Fills the 3 separated PIDs outputs (3*numpy.array[4])."""
		self.__separatedOutputs = [	DroneAttitude(*p),
									DroneAttitude(*i),
									DroneAttitude(*d)]
	
	# PRIVATE COMPUTATION
	####################################################################
	
	def __getAttitude(self):
		"""Returns the attitude to reach (DroneAttitude)."""
		if self.__mimic:
			x, y, z, roll, pitch, yaw = self.__target.getPose()
			return DroneAttitude(roll, pitch, yaw, z)
		else:
			return self.__attitude
	
	def __getSetpoint(self):
		"""Returns the setpoint to reach (DronePose)."""
		if self.__follow:
			setpoint = self.__target.getPose().toArray()
			setpoint += self.__shift
			return DronePose(*setpoint)
		else:
			return self.__setpoint
