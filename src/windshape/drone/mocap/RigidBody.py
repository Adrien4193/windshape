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
	
	Stores the pose of a Tracker published by vrpn_client_ros. Uses
	Tracker's label to find the corresponding topic name.
	
	Its attributes are the tracker's label, its pose and its tracking
	status (based on heartbeats).
	
	Attributes:
		count (int): Number of instances of RigidBody.
	
	Inherits from object.
	
	Overrides __init__, __del__, __str__.
	"""
	
	# CLASS ATTRIBUTES
	####################################################################
	
	# Total number of active class instances.
	__count = 0
	
	# STATIC METHODS
	####################################################################
	
	@staticmethod
	def getCount():
		"""Returns the number of active instances."""
		return RigidBody.__count
		
	@staticmethod
	def getTrackersList():
		"""Returns the labels of the VRPN streamed Trackers (str list).
		
		The labels are extracted from the published topics names.
		
		Topic name: /vrpn_client_node/<label>/pose
		"""
		labels = []

		# Get all topics published by ROS VRPN driver
		topics = rospy.get_published_topics('/vrpn_client_node')
		
		# Extract names of the rigid bodies from topic names
		for topic in topics:
			label = topic[0].split('/')[-2]
			labels.append(label)
		
		# Sort for display
		labels.sort()
		
		return labels
		
	# INITIALIZER AND DESTRUCTOR
	####################################################################
	
	def __init__(self, label):
		"""Stores parameters and starts updating pose.
		
		Args:
			label (str): Label of the rigid body Tracker (from Motive).
			
		Raises:
			TypeError: The label is not a string.
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
		
		# Heartbeat timeout
		self.__timeout = rospy.get_param('~tracking/timeout')
		
		rospy.logdebug('RigidBody count: %d', RigidBody.__count)
		
	def __del__(self):
		"""Closes subscriber at destruction."""
		rospy.logdebug('RigidBody destroyed: %s', self.__label)
		
		# Close ROS subscriber properly
		self.__sub.unregister()
		
		# Update instances count
		RigidBody.__count -= 1
		
		rospy.logdebug('RigidBody count %d', RigidBody.__count)
		
	# ATTRIBUTES GETTERS
	####################################################################
		
	def getLabel(self):
		"""Returns the label of the body's VRPN Tracker (str)."""
		return self.__label
		
	def getPose(self):
		"""Returns the pose of the body (DronePose)."""
		return DronePose.fromPoseStamped(self.getPoseStamped())
		
	def getPoseStamped(self):
		"""Returns the message from ROS VRPN client (PoseStamped)."""
		return self.__sub.getData()
		
	def getTopic(self):
		"""Returns the topic (str) publishing body pose."""
		return self.__topic
	
	def isTracked(self):
		"""Returns True if the RB is tracked (based on timeout)."""
		timestamp = self.getPoseStamped().header.stamp.to_sec()
		
		dt = rospy.get_time() - timestamp
		
		if dt < self.__timeout:
			return True
			
		return False
		
	# SPECIAL METHODS
	####################################################################
		
	def __str__(self):
		"""Displays the label, tracking state and pose of the RB.
		
		Structure:
		
			<label>: (<"tracked" or "lost">)
				x: <value> mm
				
				...
				
				yaw: <value> deg
		"""
		tracked = 'tracked' if self.isTracked() else 'lost'
		
		# Label and tracking state
		header = '{}: ({})\n'.format(self.__label, tracked)
		
		# Pose with 4 spaces indent
		pose = '    '+str(self.getPose()).replace('\n', '\n    ')

		return header + pose
