# ROS main library
import rospy

# ROS message to publish a 6DOF pose
from geometry_msgs.msg import PoseStamped
# MAVROS message to publish drone state
from mavros_msgs.msg import State

# Custom ROS subscriber with a buffer
from ..common.MySubscriber import MySubscriber


class MAVROSListener(object):
	"""Listen for messages from drone's FCU.
	
	Uses MAVROS topics to receive data from the drone.
	
	See wiki.ros.org/mavros for more information about MAVROS.
	
	Inherits from object.
	
	Overrides __init__, __del__.
	"""
	
	# INITIALIZER AND DESTRUCTOR
	####################################################################
	
	def __init__(self, log=None):
		"""Initializes attributes and ROS subscribers."""
		rospy.logdebug('MAVROSListener initialization')
		
		# ROS custom subscribers to store data from topics
		self.__subs = {}
		self.__initSubscribers()
		
		rospy.logdebug('MAVROSListener initialized')

	def __del__(self):
		"""Closes subscribers with debug message."""
		rospy.logdebug('MAVROSListener destruction')
		
		for sub in self.__subs.values():
			sub.unregister()
	
	# ATTRIBUTES GETTERS
	####################################################################
	
	def getState(self):
		"""Returns drone state.
		
		Topic: /mavros/state (State)
		"""
		return self.__subs['state'].getData()
	
	def getPose(self):
		"""Returns the drone pose estimated by the FCU.
		
		Topic: /mavros/local_position/pose (PoseStamped).
		"""
		return self.__subs['pose'].getData()
	
	# SUBSCRIBERS
	####################################################################
	
	def __initSubscribers(self):
		"""Subscribes to MAVROS topics to get messages from FCU."""
		
		# Drone estimated pose (from FCU)
		self.__subs['pose'] = MySubscriber('mavros/local_position/pose',
											PoseStamped)
		
		# Drone state (connected, armed, mode)
		self.__subs['state'] = MySubscriber('mavros/state',
											State)
