import math
import numpy

# ROS transformations
from tf.transformations import (euler_from_quaternion,
								quaternion_from_euler)
# ROS messages for communication
from geometry_msgs.msg import Quaternion, PoseStamped


class DroneAttitude(object):
	"""Structure used to represent the drone's attitude.
	
	roll, pitch and yaw are in radians.
	
	thrust is from 0.0 to 1.0.
	
	When converted as list or array: 0=roll, 1=pitch, 2=yaw, 3=thrust.
	
	Inherits from object.
		
	Overrides __init__, __del__, __iter__, __str__
	"""
	
	# STATIC METHODS
	####################################################################
	
	@staticmethod
	def fromPoseStamped(poseStamped):
		"""Returns DroneAttitude from a PoseStamped message."""
		q = poseStamped.pose.orientation
		roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
		
		return DroneAttitude(roll, pitch, yaw, 0)
	
	# INITIALIZER AND DESTRUCTOR
	####################################################################
	
	def __init__(self, roll=0, pitch=0, yaw=0, thrust=0):
		""" Initializes x, y, z, roll, pitch, yaw (m, rad).
		
		Args:
			roll, pitch, yaw (float): Drone orientation [rad]
			thrust (float): Drone thrust [0-1]
		"""
		self.__roll = roll
		self.__pitch = pitch
		self.__yaw = yaw
		self.__thrust = thrust
		
	def __del__(self):
		"""Does nothing special."""
		pass
		
	# ATTRIBUTES GETTERS
	####################################################################
		
	def getRoll(self):
		"""Returns the Roll float) of the attitude in [rad]."""
		return self.__roll
		
	def getPitch(self):
		"""Returns the Pitch (float) of the attitude in [rad]."""
		return self.__pitch
		
	def getYaw(self):
		"""Returns the Yaw (float) of the attitude in [rad]."""
		return self.__yaw
		
	def getThrust(self):
		"""Returns the Thrust in [0-1]."""
		return self.__thrust
		
	# ATTRIBUTES SETTERS
	####################################################################
		
	def setRoll(self):
		"""Changes the Roll float) of the attitude in [rad]."""
		self.__roll = float(value)
		
	def setPitch(self):
		"""Changes the Pitch (float) of the attitude in [rad]."""
		self.__pitch = float(value)
		
	def setYaw(self):
		"""Changes the Yaw (float) of the attitude in [rad]."""
		self.__yaw = float(value)
		
	def setThrust(self):
		"""Changes the Thrust value (float) in [0-1]."""
		self.__thrust = float(value)
		
	# CONVERSION
	####################################################################
		
	def toArray(self):
		"""Returns a numpy.array representing the attitude."""
		return numpy.array(list(self))
		
	def toPoseStamped(self):
		"""Returns the attitude as a PoseStamped message.
		
		Notes:
			Only the orientation field is taken into account (convert 
			RPY to quaternion and fill PS orientation with it).
		"""
		poseStamped = PoseStamped()
		roll, pitch, yaw, thrust = list(self)
		
		# Orientation only
		quaternion = quaternion_from_euler(roll, pitch, yaw)
		poseStamped.pose.orientation = Quaternion(*quaternion)
		
		return poseStamped
		
	def toString(self, label, shift, indent=4):
		"""Returns a string to display the attitude.
		
		Structure:
			<shift>*"space" label:
			<shift>*"space" <indent>*"space" coordinate 1
			...
		
		Args:
			label (str): The label of the attitude.
			shift (int): Number of spaces to shift the whole string.
			indent=4 (int): Number of spaces to indent pose from label.
		"""
		attitude = indent*' '+str(self).replace('\n', '\n'+indent*' ')
		string = '{}:\n{}'.format(label, attitude)
		
		return shift*' '+string.replace('\n', '\n'+shift*' ')
		
	# SPECIAL METHODS
	####################################################################
		
	def __iter__(self):
		"""Used to convert the attitude as a list."""
		yield self.__roll
		yield self.__pitch
		yield self.__yaw
		yield self.__thrust
		
	def __str__(self):
		"""Returns a string representing the attitude in deg and %."""
		roll, pitch, yaw, thrust = list(self)
		
		string = ['roll: {:.2f} deg'.format(math.degrees(roll)),
				'pitch: {:.2f} deg'.format(math.degrees(pitch)),
				'yaw: {:.2f} deg'.format(math.degrees(yaw)),
				'thrust: {:.2f} %'.format(100.0*thrust)
				]
		
		return '\n'.join(string)
