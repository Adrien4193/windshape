import math
import numpy

# ROS main library
import rospy

# Parameters
from Parameters import Parameters
# PID controller (SISO)
from PIDController import PIDController

# Receives messages from mocap system
from ..mocap.RigidBody import RigidBody


class Controller(object):
	"""MIMO controller to compute drone attitude.
	
	The attitude is computed from the X, Y, Z and Yaw errors between the
	desired setpoint and the drone pose measured by the mocap system
	using four PIDs.
	
	Input: x, y, z, roll, pitch, yaw (numpy.array[6]) [m, rad]
	
	Output: roll, pitch, yaw, thrust (numpy.array[4]) [rad, (0-1)]
	
	"Friend" class of Parameters.
	
	Inherits from object.
	
	Overrides __init__, __del__, __call__.
	"""

	# INITIALIZER AND DESTRUCTOR
	####################################################################
	
	def __init__(self):
		"""Initializes drone PIDs (x, y, z, yaw) and LP filter."""
		
		# Control attributes
		self.__parameters = Parameters()
		
		# Loads PID parameters
		pars = rospy.get_param('~control/pid')
		x, y, z, yaw = pars['x'], pars['y'], pars['z'], pars['yaw']
		
		# One PID per degree of freedom
		self.__pidX = PIDController(x['kp'], x['ki'], x['kd'],
									x['min'], x['max'], x['ff'])
										
		self.__pidY = PIDController(y['kp'], y['ki'], y['kd'],
									y['min'], y['max'], y['ff'])
										
		self.__pidZ = PIDController(z['kp'], z['ki'], z['kd'],
										z['min'], z['max'], z['ff'])	
								
		self.__pidYaw = PIDController(yaw['kp'], yaw['ki'], yaw['kd'],
									yaw['min'], yaw['max'], yaw['ff'])
		
	def __del__(self):
		"""Does nothing special."""
		pass
		
	# ATTITUDE COMPUTATION
	####################################################################

	def __call__(self, setpoint, pose):
		"""Returns attitude (numpy.array[4]) to reach setpoint.
		
		Process:
			1 - Computes X, Y, Z, Yaw error
			2 - Project XY errors on drone body frame
			3 - Compute R, P, Y, T from X, Y, Yaw, Z errors using PIDs
			4 - Returns R, P, Y, T
		
		Args:
			pose (numpy.array[6]): Current drone pose
			setpoint (numpy.array[6]): setpoint to reach
		"""
		error = setpoint - pose
		
		# Extract X, Y, Z, Yaw error from pose error
		x, y, z, yaw = list(error[0:3])+[self.__angle(error[5])]
		
		# Project XY on the body frame of the drone
		xb = x * math.cos(pose[5]) + y * math.sin(pose[5])
		yb = y * math.cos(pose[5]) - x * math.sin(pose[5])
		
		# For display
		error = numpy.array([yb, xb, yaw, z])
		self.__parameters._setError(error)
		
		# Call PIDs and mask fields with manual attitude if needed
		roll, pitch, yaw, thrust = self.__getAttitude(xb, yb, z, yaw)
		
		# Final results also used for display
		attitude = numpy.array([roll, pitch, yaw, thrust])
		self.__parameters._setControlInput(attitude)
		
		return attitude
			
	# COMMANDS
	####################################################################
		
	def reset(self):
		"""Resets integral, derivative and filter of all PIDs."""
		self.__pidX.reset()
		self.__pidY.reset()
		self.__pidZ.reset()
		self.__pidYaw.reset()
		self.__parameters._setControlInput(numpy.zeros(4))
		self.__parameters._setError(numpy.zeros(4))
		
	# ATTRIBUTES GETTERS
	####################################################################
	
	def getParameters(self):
		"""Returns the controller parameters (Parameters)."""
		return self.__parameters
		
	# PRIVATE COMPUTATIONS
	####################################################################
		
	def __angle(self, value):
		"""Returns the given angle between -pi and pi.
		
		Args:
			value (float): Angle in radians to convert.
		"""
		result = value % (2*math.pi)
		
		if result > math.pi:
			result -= (2*math.pi)
			
		return result
		
	def __getAttitude(self, xb, yb, z, yaw):
		"""Computes RPYT from pose error (m, rad)."""
		manual = self.__parameters.getManualAttitude()
		mask = self.__parameters.getMask()
		
		# Roll from -yb
		if not mask[0]:
			roll = self.__pidY(-yb)
		else:
			roll = manual.getRoll()
			self.__pidY.reset()
		
		# Pitch from xb
		if not mask[1]:
			pitch = self.__pidX(xb)
		else:
			pitch = manual.getPitch()
			self.__pidX.reset()
		
		# Yaw from yaw
		if not mask[2]:
			yaw = self.__pidYaw(yaw)
		else:
			yaw = manual.getYaw()
			self.__pidYaw.reset()
			
		# Thrust from z
		if not mask[3]:
			thrust = self.__pidZ(z)
		else:
			thrust = manual.getThrust()
			self.__pidZ.reset()
			
		return roll, pitch, yaw, thrust
