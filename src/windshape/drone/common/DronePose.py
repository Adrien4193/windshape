import math
import numpy

# ROS transformations
from tf.transformations import (euler_from_quaternion,
								quaternion_from_euler)
# ROS messages for communication
from geometry_msgs.msg import Quaternion, PoseStamped


class DronePose(object):
	"""Structure used to represent a 6DOF pose.
	
	x, y and z coordinates are in meters.
	
	roll, pitch and yaw are in radians.
	
	When converted as list or array: 0=x, ..., 5=yaw
	
	Inherits from object.
		
	Overrides __init__, __del__, __iter__, __str__
	"""
	
	# STATIC METHODS
	####################################################################
	
	@staticmethod
	def fromPoseStamped(poseStamped):
		"""Returns a DronePose from a PoseStamped message."""
		# Position
		x = poseStamped.pose.position.x
		y = poseStamped.pose.position.y
		z = poseStamped.pose.position.z
		
		# Orientation
		q = poseStamped.pose.orientation
		roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
		
		return DronePose(x, y, z, roll, pitch, yaw)
	
	# INITIALIZER AND DESTRUCTOR
	####################################################################
	
	def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
		"""Initializes x, y, z, roll, pitch, yaw (m, rad).
		
		Args:
			x, y, z (float): Drone position [m]
			roll, pitch, yaw (float): Drone orientation [rad]
		"""
		self.__x = x
		self.__y = y
		self.__z = z
		self.__roll = roll
		self.__pitch = pitch
		self.__yaw = yaw
		
	def __del__(self):
		"""Does nothing special."""
		pass
		
	# ATTRIBUTES GETTERS
	####################################################################
	
	def getX(self):
		"""Returns the X coordinate of the pose in [m]."""
		return self.__x
		
	def getY(self):
		"""Returns the Y coordinate of the pose in [m]."""
		return self.__y
		
	def getZ(self):
		"""Returns the Z coordinate of the pose in [m]."""
		return self.__z
		
	def getRoll(self):
		"""Returns the Roll coordinate of the pose in [rad]."""
		return self.__roll
		
	def getPitch(self):
		"""Returns the Pitch coordinate of the pose in [rad]."""
		return self.__pitch
		
	def getYaw(self):
		"""Returns the Yaw coordinate of the pose in [rad]."""
		return self.__yaw
		
	# ATTRIBUTES SETTERS
	####################################################################
	
	def setX(self, value):
		"""Changes the X coordinate (float) of the pose in [m]."""
		self.__x = float(value)
		
	def setY(self):
		"""Changes the Y coordinate (float) of the pose in [m]."""
		self.__y = float(value)
		
	def setZ(self):
		"""Changes the Z coordinate (float) of the pose in [m]."""
		self.__z = float(value)
		
	def setRoll(self):
		"""Changes the Roll coordinate (float) of the pose in [rad]."""
		self.__roll = float(value)
		
	def setPitch(self):
		"""Changes the Pitch coordinate (float) of the pose in [rad]."""
		self.__pitch = float(value)
		
	def setYaw(self):
		"""Changes the Yaw coordinate (float) of the pose in [rad]."""
		self.__yaw = float(value)
		
	# CONVERSION
	####################################################################
		
	def toArray(self):
		"""Returns a numpy.array representing the pose."""
		return numpy.array(list(self))
		
	def toPoseStamped(self):
		"""Returns the pose as a PoseStamped message."""
		poseStamped = PoseStamped()
		x, y, z, roll, pitch, yaw = list(self)
		
		# Position
		poseStamped.pose.position.x = x
		poseStamped.pose.position.y = y
		poseStamped.pose.position.z = z
		
		# Orientation
		quaternion = quaternion_from_euler(roll, pitch, yaw)
		poseStamped.pose.orientation = Quaternion(*quaternion)
		
		return poseStamped
		
	def toString(self, label, shift=0, indent=4):
		"""Returns a string to display the pose.
		
		Structure:
			<shift>*"space" label:
			<shift>*"space" <indent>*"space" coordinate 1
			...
		
		Args:
			label (str): The label of the pose.
			shift (int): Number of spaces to shift the whole string.
			indent=4 (int): Number of spaces to indent pose from label.
		"""
		pose = indent*' '+str(self).replace('\n', '\n'+indent*' ')
		string = '{}:\n{}'.format(label, pose)
		
		return shift*' '+string.replace('\n', '\n'+shift*' ')
		
	# SPECIAL METHODS
	####################################################################
		
	def __iter__(self):
		"""Used to convert the pose as a list."""
		yield self.__x
		yield self.__y
		yield self.__z
		yield self.__roll
		yield self.__pitch
		yield self.__yaw
		
	def __str__(self):
		"""Returns a string representing the pose in mm and deg."""
		x, y, z, roll, pitch, yaw = list(self)
		
		string = [	'x: {:.2f} mm'.format(1000*x),
					'y: {:.2f} mm'.format(1000*y),
					'z: {:.2f} mm'.format(1000*z),
					'roll: {:.2f} deg'.format(math.degrees(roll)),
					'pitch: {:.2f} deg'.format(math.degrees(pitch)),
					'yaw: {:.2f} deg'.format(math.degrees(yaw))]
		
		return '\n'.join(string)
