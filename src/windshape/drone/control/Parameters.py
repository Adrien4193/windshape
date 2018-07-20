# ROS main library
import rospy

# RPYT attitude with facilities
from ..common.DroneAttitude import DroneAttitude
# 6DOF pose with facilities
from ..common.DronePose import DronePose


class Parameters(object):
	"""Control parameters used in offboard.
	
	Inherits from: object.
	
	Overrides: __init__, __del__.
	"""
	
	# CLASS ATTRIBUTES
	####################################################################
	
	MODES = ['onboard', 'offboard']
	
	# INITIALIZER AND DESTRUCTOR
	####################################################################
	
	def __init__(self):
		"""Loads parameters."""
		
		# Control customization
		self.__mode = rospy.get_param('~control/mode')
		self.__mask = rospy.get_param('~control/mask')
		self.__manualAttitude = DroneAttitude()
		
		# Display
		self.__controlInput = DroneAttitude()
		self.__error = DroneAttitude()
		
	def __del__(self):
		"""Does nothing special."""
		pass
	
	# PARAMETERS GETTERS
	####################################################################
	
	def getControlInput(self):
		"""Returns the last control input sent (DroneAttitude).
		
		The control input is an attitude (roll, pitch, yaw, thrust).
		
		Notes:
			Is zero if no offboard control is performed.
			Does not compute anything, just displays.
		"""
		return self.__controlInput
		
	def getError(self):
		"""Returns the last error (setpoint - pose) (DroneAttitude).
		
		The error is composed of XYZYaw (X and Y are projected on the
		body frame).
		"""
		return self.__error
		
	def getManualAttitude(self):
		"""Returns the last attitude entered by user (DroneAttitude).
		
		Replace controller output if mask set as True.
		"""
		return self.__manualAttitude
		
	def getMask(self):
		"""Returns the control mask (bool[4]).
		
		Order: roll, pitch, yaw, thrust
		
		True = Replace controller output field by manual attitude.
		False = Use controller output.
		"""
		return self.__mask
		
	def getMode(self):
		"""Returns the current position control mode (str).
		
		"offboard" = Control done offboard using attitude setpoints.
		"onboard" = Control done onboard using position setpoints.
		"""
		return self.__mode
		
	# PARAMETERS SETTERS
	####################################################################
		
	def setManualAttitude(self, roll, pitch, yaw, thrust):
		"""Assigns manually an attitude to the drone.
		
		Args:
			roll, pitch, yaw: Orientation of the drone [rad]
			thrust: Thrust of the drone [0-1]
			
		Note:
			Needs to be armed with OFFBOARD flight and control modes.
			Will have an effect only if the corresponding component is
			set to True in the mask.
		"""
		self.__manualAttitude = DroneAttitude(roll, pitch, yaw, thrust)
		
	def setMask(self, roll, pitch, yaw, thrust):
		"""Enable or disable component of the drone attitude control.
		
		True = Replace controller output field by manual attitude.
		False = Use controller output.
		
		Args:
			roll, pitch, yaw, thrust (bool): True to replace component.
		"""
		self.__mask = [roll, pitch, yaw, thrust]
		
	def setMode(self, mode):
		"""Changes the position control mode.
		
		"offboard" = Control done offboard using attitude setpoints.
		"onboard" = Control done onboard using position setpoints.
		
		Args:
			mode (str): The new control mode (see above)
		"""
		self.__mode = mode
		
	# PROTECTED CONTROLLER SETTERS
	####################################################################
	
	def _setControlInput(self, attitude):
		"""Fills control input field (numpy.array[4])."""
		self.__controlInput = DroneAttitude(*attitude)
		
	def _setError(self, error):
		"""Fills error field (numpy.array[4])."""
		self.__error = DroneAttitude(*error)
