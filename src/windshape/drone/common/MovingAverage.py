# ROS main library
import rospy


class MovingAverage(object):
	"""Simple moving average filter.
	
	The filter is called as a function.
	
	The response y of a signal x is based on the following relationship:
	
	y[k] = x[k] + x[k-1] + ... + x[k-n+1]
	
	where a is the parameter given to __init__. 
	
	Inherits from object.
	
	Overrides: __init__, __del__, __call__.
	"""
	
	def __init__(self, n, initValue):
		"""Stores a and initializes previous value.
		
		Arg:
			n (int): Number of points (must be >= 1).
			initValue (object with __add__): Inititalization.
		"""
		if not isinstance(n, int) or n < 1:
			rospy.logfatal('n = %s is not an int >= 1', n)
		
		# Loads weight of previous output
		self.__n = n
		
		# Initializes previous values
		self.__x = (n-1) * [initValue]
		
		# For reset
		self.__resetValue = (n-1) * [initValue]
		
	def __del__(self):
		"""Does nothing special."""
		pass
	
	def __call__(self, x):
		"""Applies the filter on the given point.
		
		Arg:
			x (object with __add__): Current point of the input signal
		"""
		y = (x + sum(self.__x)) / self.__n
		
		self.__x.append(x)
		del self.__x[0]
		
		return y
		
	#
	# Public methods to reset the filter.
	#
	
	def reset(self):
		"""Resets the memory to init value."""
		self.__x = self.__resetValue
