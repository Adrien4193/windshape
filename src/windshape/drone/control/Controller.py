import math
import numpy

# ROS main library
import rospy

# Parameters
from ControlParameters import ControlParameters
# PID controller (SISO)
from PIDController import PIDController

# Receives messages from mocap system
from ..mocap.RigidBody import RigidBody

# Low-pass filter
from ..common.LowPassFilter import LowPassFilter


class Controller(object):
	"""MIMO controller to compute drone attitude.
	
	The attitude is computed from the X, Y, Z and Yaw errors between the
	desired setpoint and the drone pose measured by the mocap system
	using four PIDs.
	
	Input: x, y, z, roll, pitch, yaw (numpy.array[6]) [m, rad]
	
	Output: roll, pitch, yaw, thrust (numpy.array[4]) [rad, (0-1)]
	
	"Friend" class of ControlParameters.
	
	Inherits from object.
	
	Overrides __init__, __del__, __call__.
	"""

	# INITIALIZER AND DESTRUCTOR
	####################################################################
	
	def __init__(self):
		"""Initializes drone PIDs (x, y, z, yaw) and LP filter."""
		
		# Control attributes
		self.__parameters = ControlParameters()
		
		# Filter for manual position setpoint
		self.__filter = LowPassFilter(200, numpy.zeros(6))
		
		# Loads PID parameters
		pars = rospy.get_param('~control/pid')
		r, p, y, t = (pars['roll'], pars['pitch'], pars['yaw'],
						pars['thrust'])
		
		# Roll, pitch, yaw, thrust
		self.__pids = [	PIDController(r['kp'], r['ki'], r['kd'],
									r['min'], r['max'], r['ff']),
		
						PIDController(p['kp'], p['ki'], p['kd'],
									p['min'], p['max'], p['ff']),
									
						PIDController(y['kp'], y['ki'], y['kd'],
									y['min'], y['max'], y['ff']),
									
						PIDController(t['kp'], t['ki'], t['kd'],
									t['min'], t['max'], t['ff'])]
		
	def __del__(self):
		"""Does nothing special."""
		pass
		
	# ATTRIBUTES GETTERS
	####################################################################
	
	def getParameters(self):
		"""Returns the controller parameters (ControlParameters)."""
		return self.__parameters
		
	# COMMANDS
	####################################################################
		
	def reset(self):
		"""Resets PIDs and display when controller is disabled."""
		for pid in self.__pids:
			pid.reset()
		self.__parameters._setControlInput(numpy.zeros(4))
		self.__parameters._setError(numpy.zeros(4))
		self.__parameters._setSetparatedOutputs(*(3*[numpy.zeros(4)]))
		
	# ATTITUDE COMPUTATION (CALL)
	####################################################################

	def __call__(self, setpoint, pose, estimate):
		"""Returns attitude (numpy.array[4]) to reach setpoint.
		
		Process:
			1 - Computes X, Y, Z, Yaw error.
			2 - Project XY errors on drone body frame.
			3 - Sets yaw feed-forward as desired yaw from estimate.
			3 - Computes R, P, Y, T from X, Y, Yaw, Z errors using PIDs.
			4 - Returns R, P, Y, T.
		
		Args:
			setpoint (numpy.array[6]): setpoint to reach.
			pose (numpy.array[6]): Current real drone pose.
			estimate (float): Pose estimated by FCU (can be shifted).
		"""
		setpoint = self.__parameters.getSetpoint()
		
		# Filters manual setpoint from drone pose
		if self.__parameters.getTask() == 'reach_setpoint':
			setpoint = self.__filter(setpoint)
		else:
			self.__filter.reset(pose)
		
		# Computes pose error
		error = setpoint - pose
		
		# Projects XY from global frame to body frame of the drone
		x, y = error[0:2]
		xb = x * math.cos(pose[5]) + y * math.sin(pose[5])
		yb = y * math.cos(pose[5]) - x * math.sin(pose[5])
		
		# Computes yaw feed forward in drone estimated frame
		yaw = setpoint[5] - pose[5] + estimate[5]
		self.__pids[2].setFeedForward(setpoint[5])
		
		# -yb -> roll, xb -> pitch, yaw -> yaw, z -> thrust
		error = numpy.array([-yb, xb, error[5], error[2]])
		
		# Calls PIDs and masks fields with manual attitude if needed
		attitude = self.__computeAttitude(error)
		
		# Display
		self.__record(attitude, error)
		
		return attitude
		
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
		
	def __computeAttitude(self, error):
		"""Computes RPYT from pose error (m, rad)."""
		mask = self.__parameters.getMask()
		
		# Initializes with manual attitude
		attitude = list(self.__parameters.getAttitude())
		
		# Roll, pitch, yaw, thrust
		for axis in range(4):
			
			# Replaces axis with controller value if not masked
			if not mask[axis]:
				attitude[axis] = self.__pids[axis](error[axis])
			else:
				self.__pids[axis].reset()
			
		return numpy.array(attitude)
		
	def __record(self, attitude, error):
		"""Records outputs in parameters."""
		# Total output RPYT
		self.__parameters._setControlInput(attitude)
		
		# Error RPYT
		self.__parameters._setError(error)
		
		# Separated PID contributions
		attitudes = 3*[numpy.zeros(4)]
		for axis in range(4):
			outputs = self.__pids[axis].getSeparatedOutputs()
			for output in range(3):
				attitudes[output][axis] = outputs[output]
		self.__parameters._setSetparatedOutputs(*attitudes)
