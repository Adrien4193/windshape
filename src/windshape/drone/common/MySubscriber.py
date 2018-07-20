# ROS main library
import rospy


class MySubscriber(rospy.Subscriber):
	"""Custom ROS subscriber to store data from a ROS topic.
	
	Overrides rospy.Subscriber to get and memorize the data published on
	a given ROS topic from its name ("/node/...") and its type
	(ROS message).
	
	Inherits:
		rospy.Subscriber
	
	Overrides:
		__init__, __del__.
	"""
	
	##################### INITIALIZER, DESTRUCTOR ######################
	
	def __init__(self, topic, message_type):
		"""Calls parent constructor and init attributes. 
		
		Args:
			topic (str): ROS topic that publishes the data.
			message_type (ROS message): Expected ROS message type.
			
		Raises:
			TypeError: topic is not a string
		"""
		rospy.logdebug('New subscription to %s', topic)
		
		# Safety
		if not isinstance(topic, str):
			message = str(topic)+'is not a string'
			rospy.logfatal(message)
			raise TypeError(message)
		
		# ROS topic to listen
		self.__topic = topic
		
		# Call parent constructor
		super(MySubscriber, self).__init__(topic, message_type,
											self.__data_cb)
		# Initialize buffer
		self.__buffer = message_type()
		
	def __del__(self):
		"""Just logs debug message."""
		rospy.logdebug('Unregister from %s', self.__topic)
		
	######################## ATTRIBUTES GETTERS ########################
	
	def getData(self):
		"""Returns the last message received."""
		return self.__buffer
		
	def getTopic(self):
		"""Returns the topic subscribed (str)."""
		return self.__topic
		
	######################### PRIVATE METHODS ##########################
	
	def __data_cb(self, data):
		"""Callback when a new message is published.
		
		Called each time a message is published on the topic given to
		the contructor with the message as argument.
		"""
		self.__buffer = data
