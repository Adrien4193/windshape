# ROS main library
import rospy

# MAVROS services commands 
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


class MAVROSClient(object):
	"""Sends commands to the drone.
	
	Call MAVROS services to send commands to the drone.
	
	See wiki.ros.org/mavros for more information about MAVROS.
	
	Inherits from object.
	
	Overrides __init__, __del__.
	"""
	
	# CLASS ATTRIBUTES
	####################################################################
	
	# Flight modes (PX4)
	PX4_FLIGHT_MODES = ['MANUAL',
						'ACRO',
						'ALTCTL',
						'POSCTL',
						'OFFBOARD',
						'STABILIZED',
						'RATTITUDE',
						'AUTO.MISSION',
						'AUTO.LOITER',
						'AUTO.RTL',
						'AUTO.LAND',
						'AUTO.RTGS',
						'AUTO.READY',
						'AUTO.TAKEOFF']

	# INITIALIZER AND DESTRUCTOR
	####################################################################
	
	def __init__(self):
		"""Just logs debug messages."""
		rospy.logdebug('MAVROSClient initialization')
		rospy.logdebug('MAVROSClient initialized')
		
	def __del__(self):
		"""Just logs debug message."""
		rospy.logdebug('MAVROSClient destruction')
		
	# COMMANDS
	####################################################################
		
	def callArming(self, command):
		"""Arms the drone with MAVROS service cmd/arming."""
		rospy.loginfo('Arming' if command else 'Disarming')
		self.__callService('/mavros/cmd/arming', CommandBool,
							command)
		
	def callLand(self):
		"""Auto-land the drone with MAVROS service cmd/land."""
		rospy.loginfo('Landing ...')
		self.__callService('/mavros/cmd/land', CommandTOL,
							0, 0, 0, 0, 0)
		
	def callSetMode(self, mode):
		"""Changes flight mode with MAVROS service set_mode."""
		rospy.loginfo('Setting mode to '+str(mode))
		self.__callService('/mavros/set_mode', SetMode,
							0, mode)
		
	def callTakeoff(self):
		"""Auto-takeoff the drone with MAVROS service cmd/takeoff."""
		rospy.loginfo('Takeoff ...')
		self.__callService('/mavros/cmd/takeoff', CommandTOL,
							0, 0, 0, 0, 1.0)
		
	# ROS SERVICES CALL
	####################################################################
	
	def __callService(self, serviceName, commandType, *args):
		"""Call ROS service with all safeties."""
		
		# Service timeout
		tout = rospy.get_param('~update/service_wait_timeout')
		
		# Wait for service and return if not available
		try:
			rospy.wait_for_service(serviceName, tout)
		except rospy.ROSException as e:
			rospy.logerr(e)
			return
		
		# Call service
		try:
			service = rospy.ServiceProxy(serviceName, commandType)
			service(*args)
		except rospy.ServiceException as e:
			rospy.logerr(e)
