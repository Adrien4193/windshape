# ROS main library
import rospy

# ROS transforms
from tf.transformations import quaternion_from_euler

# ROS message to publish geometric data
from geometry_msgs.msg import Point, PoseStamped, Quaternion
# MAVROS message to publish a Thrust
from mavros_msgs.msg import Thrust


class MAVROSPublisher(object):
	"""Send MAVLink messages to the drone.
	
	Publishes messages on MAVROS (source: wiki.ros.org/mavros) topics
	to send them to the drone.
	
	Inherits from object.
	
	Overrides __init__, __del__.
	"""
	
	# INITIALIZER AND DESTRUCTOR
	####################################################################
	
	def __init__(self):
		"""Initializes publishers."""
		rospy.logdebug('MAVROSPublisher initialization')
		
		# Publishers to send data through MAVROS topics
		self.__pubs = {}
		self.__initPublishers()
											
		rospy.logdebug('MAVROSPublisher initialized')
		
	def __del__(self):
		"""Closes publishers with debug message."""
		rospy.logdebug('MAVROSPublisher destruction')
		
		for pub in self.__pubs.values():
			pub.unregister()
			
	# DATA PUBLISHING
	####################################################################
	
	def sendMocapPose(self, poseStamped):
		"""Publishes mocap pose on "/mavros/mocap/pose".
		
		Must be called continuously (FCU crashes above 240 Hz).
		
		Uses MAVLink ATT_POS_MOCAP message (see mavros_extras plugin
		"mocap_pose_estimate").
		
		Notes:
			The PX4 parameter "group estimator" must be set to "LPE" to
			take this external heading into account.
		
		Args:
			poseStamped (PoseStamped): Message from the mocap system.
		"""
		if not isinstance(poseStamped, PoseStamped):
			rospy.logerr('%s is not a PoseStamped', poseStamped)
		
		self.__pubs['mocap'].publish(poseStamped)

	def sendSetpointAttitude(self, poseStamped):
		"""Publishes setpoint on mavros/setpoint_attitude/attitude.
		 
		Must be called at minimum 2Hz to enable OFFBOARD mode.
		
		Uses MAVLink SET_ATTITUDE_TARGET message (see mavros plugin
		"setpoint_attitude").
		
		Notes:
			Only orientation field (Quaternion) is taken into account.
			MAVROS parameter "/mavros/setpoint_attitude/use_quaternion"
			must be set to True.
		
		Args:
			poseStamped (PoseStamped): Attitude message to send.
		"""
		if not isinstance(poseStamped, PoseStamped):
			rospy.logerr('%s is not a PoseStamped', poseStamped)
		
		self.__pubs['attitude'].publish(poseStamped)
		
	def sendSetpointThrust(self, thrust):
		"""Publishes setpoint on /mavros/setpoint_attitude/thrust.
		 
		Must be called at minimum 2Hz to enable OFFBOARD flight mode.
		
		Uses MAVLink SET_ATTITUDE_TARGET message (see mavros plugin
		"setpoint_attitude").
		
		Notes:
			Uses the same MAVLink message as attitude.
		
		Args:
			thrust (float): Thrust value (0-1).
		"""
		if not isinstance(thrust, (float, int)):
			rospy.logerr('%s is not a number', thrust)
		
		thrust = Thrust(thrust=thrust)
		self.__pubs['thrust'].publish(thrust)
		
	def sendSetpointPosition(self, poseStamped):
		"""Publishes setpoint on /mavros/setpoint_position/local.
		
		Must be called at minimum 2Hz to enable OFFBOARD mode.
		
		Uses MAVLink SET_POSITION_TARGET_LOCAL_NED message (see mavros
		plugin "setpoint_position").
		
		Needs external heading ! (Mocap, vision, GPS)
		
		Notes:
			Only yaw is extracted from orientation field.
		
		Args:
			poseStamped (PoseStamped): Position to reach.
		"""
		if not isinstance(poseStamped, PoseStamped):
			rospy.logerr('%s is not a PoseStamped', poseStamped)
		
		self.__pubs['position'].publish(poseStamped)
		
	# INITIALIZATION
	####################################################################
	
	def __initPublishers(self):
		"""Creates publishers to send data through MAVROS topics."""
		
		# Sends external heading to the drone to compute local estimate
		self.__pubs['mocap'] = rospy.Publisher(
								'mavros/mocap/pose',
								PoseStamped, queue_size=10)
											
		# Sends position setpoint to reach using fcu position controller
		self.__pubs['position'] = rospy.Publisher(
								'mavros/setpoint_position/local',
								PoseStamped, queue_size=10)
											
		# Sends attitude setpoint to reach using fcu attitude controller
		self.__pubs['attitude'] = rospy.Publisher(
								'mavros/setpoint_attitude/attitude',
								PoseStamped, queue_size=10)
											
		self.__pubs['thrust'] = rospy.Publisher(
								'mavros/setpoint_attitude/thrust',
								Thrust, queue_size=10)
