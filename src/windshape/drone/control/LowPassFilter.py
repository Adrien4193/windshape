# ROS main library
import rospy


class LowPassFilter(object):
	"""Simple low-pass filter.
	
	The response y of a signal x is based on the following relationship:
	
	y[n] = (x[n] + a*y[n-1]) / (a+1)
	
	where a is the parameter given to __init__. 
	
	Inherits from object.
	
	Overrides: __init__, __del__, __call__.
	"""
	
	###################### INITIALIZER, DESTRUCTOR #####################
	
	def __init__(self, a, initValue):
		"""Stores a and initializes previous value.
		
		Arg:
			a (float): Weight of x[n-1] (a in description)
			initValue (object with __add__): Inititalization of y[n-1]
		"""
		# Safety
		if a < 0:
			rospy.logfatal('%s is < 0', a)
		
		# Loads weight of previous output
		self.__a = a
		
		# Initializes y[n-1]
		self.__y_1 = initValue
		
		# For reset
		self.__resetValue = initValue
		
	def __del__(self):
		"""Does nothing special."""
		pass
	
	############################# CALLER ###############################
	
	def __call__(self, x):
		"""Applies the filter on the given point.
		
		Arg:
			x (object with __add__): Current point of the input signal
		"""
		a = self.__a
		y_1 = self.__y_1
		
		# Computes next value of y
		y = (x + a*y_1) / (a+1)
		
		# Updates previous value of y
		self.__y_1 = y
		
		return y
		
	# COMMANDS
	####################################################################
	
	def reset(self):
		"""Resets the memory to init value."""
		self.__y_1 = self.__resetValue
