# ROS main library
import rospy


class MySubscriber(rospy.Subscriber):
	"""Custom ROS subscriber to store data from a ROS topic.
	
	Overrides rospy.Subscriber to get and memorize the data published on
	a given ROS topic from its name ("/node/...") and its type
	(ROS message).
	
	Inheritsfrom: rospy.Subscriber
	
	Overrides: __init__, __del__.
	"""
	
	def __init__(self, topic, messageType):
		"""Calls parent constructor and init attributes. 
		
		Args:
			topic (str): ROS topic that publishes the data.
			message_type (ROS message): Expected ROS message type.
		"""
		rospy.logdebug('New subscription to %s', topic)
		
		# Safety
		if not isinstance(topic, str):
			rospy.logfatal('%s is not a string', message)
		
		# Attributes
		self.__topic = topic
		self.__buffer = messageType()
		
		# Call parent constructor
		super(MySubscriber, self).__init__(topic, messageType,
											self.__data_cb)
		
	def __del__(self):
		"""Just logs debug message."""
		rospy.logdebug('Unregister from %s', self.__topic)
		
	#
	# Public methods to get the messages.
	#
	
	def getData(self):
		"""Returns the last message received (ROS message)."""
		return self.__buffer
		
	def getTopic(self):
		"""Returns the topic subscribed (str)."""
		return self.__topic
		
	#
	# Privates methods to receive the messages from ROS.
	#
	
	def __data_cb(self, data):
		"""Called each time a new message is published."""
		self.__buffer = data
