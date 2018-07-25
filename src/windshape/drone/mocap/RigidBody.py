# ROS main library
import rospy
# ROS message to publish a 6DOF pose
from geometry_msgs.msg import PoseStamped

# Custom ROS subscriber with a buffer
from ..common.MySubscriber import MySubscriber
# 6DOF pose with facilities
from ..common.DronePose import DronePose


class RigidBody(object):
	"""6DOF rigid body associated to a VRPN Tracker.
	
	Used to get the pose of a streamed VRPN Tracker with vrpn_client_ros
	(see: http://wiki.ros.org/vrpn_client_ros). Uses Tracker's label to
	find the corresponding topic name.
	
	Inherits from: object.
	
	Overrides: __init__, __del__, __str__.
	"""
	
	## Total number of class instances.
	__count = 0
	
	@staticmethod
	def getCount():
		"""Returns the number of RigidBody instances."""
		return RigidBody.__count
		
	@staticmethod
	def getTrackersList():
		"""Returns the labels of the streamed VRPN Trackers (str list).
		
		The labels are extracted from the names of the topics published
		by vrpn_client_ros.
		
		Structure of topic name: "/vrpn_client_node/<label>/pose"
		"""
		labels = []

		# Gets all topics published by ROS VRPN driver
		topics = rospy.get_published_topics('/vrpn_client_node')
		
		# Extracts names of the rigid bodies from topic names
		for topic in topics:
			if 'pose' in topic:
				label = topic[0].split('/')[-2]
				labels.append(label)
		labels.sort()
		
		return labels
	
	def __init__(self, label):
		"""Stores parameters and starts updating pose.
		
		Args:
			label (str): Label of the rigid body Tracker (from Motive).
		"""
		rospy.logdebug('New RigidBody created: %s', label)
		
		# Safety
		if not isinstance(label, str):
			rospy.logfatal('%s is not a string', label)
			
		# Update class instances count
		RigidBody.__count += 1
		
		# Attributes
		self.__label = label
		self.__topic = '/vrpn_client_node/'+label+'/pose'
		self.__sub = MySubscriber(self.__topic, PoseStamped)
		
		# Heartbeat timeout (lost if no publication since...)
		self.__timeout = rospy.get_param('~tracking/timeout')
		
		rospy.logdebug('RigidBody count: %d', RigidBody.__count)
		
	def __del__(self):
		"""Closes subscriber at destruction."""
		rospy.logdebug('RigidBody destroyed: %s', self.__label)
		
		# Destroys subscriber
		self.__sub.unregister()
		
		# Updates instances count
		RigidBody.__count -= 1
		
		rospy.logdebug('RigidBody count %d', RigidBody.__count)
		
	def __str__(self):
		"""Displays the label, tracking state and pose of the RB.
		
		Structure:
		
			<label>: (<"tracked" or "lost">)
				x: <value> mm
				y: <value> mm
				z: <value> mm
				roll: <value> deg
				pitch: <value> deg
				yaw: <value> deg
		"""
		tracked = 'tracked' if self.isTracked() else 'lost'
		
		# Label and tracking state
		header = '{} ({})\n'.format(self.__label, tracked)
		
		return self.getPose().toString(header)
	
	#
	# Public methods to access attributes.
	#
	
	def getLabel(self):
		"""Returns the label of the body's VRPN Tracker (str)."""
		return self.__label
		
	def getPose(self):
		"""Returns the pose of the body (DronePose)."""
		return DronePose.fromPoseStamped(self.getPoseStamped())
		
	def getPoseStamped(self):
		"""Returns the raw pose message from client (PoseStamped)."""
		return self.__sub.getData()
		
	def getTopic(self):
		"""Returns the topic (str) publishing the RB's pose."""
		return self.__topic
	
	def isTracked(self):
		"""Returns True if the RB is tracked (based on timeout)."""
		timestamp = self.getPoseStamped().header.stamp.to_sec()
		dt = rospy.get_time() - timestamp
		
		if dt < self.__timeout:
			return True
			
		return False
