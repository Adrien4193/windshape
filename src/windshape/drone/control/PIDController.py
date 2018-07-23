import time
import math

# ROS main library
import rospy

# Low-pass filters
from ..common.LowPassFilter import LowPassFilter


class PIDController(object):
	"""Simple SISO PID controller with anti-windup and manual reset.
	
	The parameters (gains, saturation) are given to the constructor.
	
	Inherits from object.
	
	Overrides: __init__, __del__, __call__.
	"""
	
	# INITIALIZER AND DESTRUCTOR
	####################################################################
	
	def __init__(self, kp, ki, kd, umin, umax, ff):
		"""Initializes parameters, integral and previous error.
		
		Args:
			kp (float): Proportional gain
			ki (float): Integral gain
			kd (float): Derivative gain
			umin (float): Minimum value of the output
			umax (float): Maximum value of the output
			ff (float): Feed-forward to add to the output
		"""
		# P, I and D gains
		self.__gainP = kp
		self.__gainI = ki
		self.__gainD = kd
		
		# Saturation
		self.__max = umax
		self.__min = umin
		
		# Feed-forward
		self.__feedForward = ff
		
		# Derivative filter
		self.__filter = LowPassFilter(0.5, 0.0)
		
		# For integral and derivative computation
		self.__integral = 0.0
		self.__previousError = 0.0
		
		# Time step computation
		self.__previousCall = None
		
		# Separated outputs for display
		self.__separatedOutputs = 3*[0.0]
		
	def __del__(self):
		"""Does nothing special."""
		pass
		
	# ATTRIBUTES
	####################################################################
	
	def getError(self):
		"""Returns the last error computed (float)."""
		return self.__previousError
		
	def getIntegral(self):
		"""Returns the current integral (float)."""
		return self.__integral
		
	def getSeparatedOutputs(self):
		"""Returns the last output computed (P, I, D)."""
		return self.__separatedOutputs
		
	# COMMANDS
	####################################################################
	
	def reset(self):
		"""Resets the controller's parameters."""
		self.__previousCall = None
		
	def setFeedForward(self, value):
		"""Changes the feed forward value (float)."""
		self.__feedForward = value
		
	# COMPUTATION
	####################################################################
		
	def __call__(self, error):
		"""Returns the control input from error.
		
		Uses anti-windup for integral and filters error for derivative.
		
		Args:
			error (float): Desired value - Measured value
		"""
		# Time step
		dt = self.__getTimeStep()
		
		# Uses first call as reference
		if dt == 0:
			self.__reset(error)
			return 0
		
		# Intergral term
		if self.__gainI != 0:
			self.__integral += error * dt
		
		# Derivative term
		derivative = self.__filter(error - self.__previousError) / dt
		self.__previousError = error
		
		# Sums FF, P, I and D terms
		output = self.__feedForward
		output += self.__gainP * error
		output += self.__gainI * self.__integral
		output += self.__gainD * derivative
		
		# Saturation and anti-windup
		output = self.__checkSaturation(output)
		
		# Display
		self.__record(error, derivative)
		
		return output
		
	# PRIVATE COMPUTATION
	####################################################################
		
	def __checkSaturation(self, output):
		"""Returns the output (float) in the saturation limits."""
		delta1 = output - self.__max
		delta2 = output - self.__min
		
		# Saturation MAX
		if output > self.__max:
			output = self.__max
			
			# Anti-windup
			if self.__gainI != 0:
				self.__integral -= delta1 / self.__gainI
		
		# Saturation MIN
		elif output < self.__min:
			output = self.__min
			
			# Anti-windup
			if self.__gainI != 0:
				self.__integral -= delta2 / self.__gainI
		
		return output
		
	def __getTimeStep(self):
		"""Returns the time step (float) between two calls."""
		if self.__previousCall is None:
			dt = 0
		else:
			dt = rospy.get_time() - self.__previousCall
			self.__previousCall = rospy.get_time()
			
		return dt
		
	def __record(self, error, derivative):
		"""Records control input in class attribute."""
		self.__separatedOutputs = [self.__gainP * error,
									self.__gainI * self.__integral,
									self.__gainD * derivative]
		
	def __reset(self, error):
		"""Initializes controller for first call."""
		self.__previousCall = rospy.get_time()
		self.__integral = 0
		self.__previousError = error
		self.__separatedOutputs = 3*[0]
		self.__filter.reset(0)
