import numpy

# ROS main library
import rospy

# Receives messages from mocap system
from ..mocap.RigidBody import RigidBody

# RPYT attitude with facilities
from ..common.DroneAttitude import DroneAttitude
# 6DOF pose with facilities
from ..common.DronePose import DronePose


class ControlParameters(object):
	"""Control parameters used in offboard position control.
	
	The parameters are evaluated as follows:
	
	- Offboard: Use offboard custom position controller. Else use PX4.
	
	- Target: A rigid body defined on the mocap system.
	
	- Manual setpoint: XYZYaw pose to reach.
	
	- Follow: Follow or not the target (with a shift to save user).
	
	- Setpoint: The position to reach (XYZYaw). Can be either the target
		pose or manual setpoint depending on "follow".
	
	- Mask: Mask roll, pitch, yaw or thrust of the controller output and
		replace by "attitude". "offboard" must be set to have an effect.
	
	- Attitude: The attitude to reach if the controller output is
		masked. Can be the target attitude or a manual one.
	
	"Friend" class: Controller
	
	Inherits from: object.
	
	Overrides: __init__, __del__.
	"""
	
	def __init__(self):
		"""Loads parameters from config file."""
		rospy.logdebug('Loading control parameters')
		
		# Target
		self.__target = RigidBody(rospy.get_param('~control/target'))
		self.__follow = rospy.get_param('~control/follow')
		self.__shift = numpy.array(rospy.get_param('~control/shift'))
		self.__mimic = rospy.get_param('~control/mimic')
		
		# Manual pose and attitude to reach
		self.__setpoint = DronePose()
		self.__attitude = DroneAttitude()
		
		# Use or not offboard position controller
		self.__offboard = rospy.get_param('~control/offboard')
		self.__mask = rospy.get_param('~control/mask')
		
		# Display
		self.__controlInput = DroneAttitude()
		self.__error = DroneAttitude()
		self.__separatedOutputs = 3*[DroneAttitude()]
		
		rospy.logdebug('Control parameters loaded')
		
	def __del__(self):
		"""Does nothing special."""
		pass
		
	def __str__(self):
		"""Display the parameters."""
		return '\n'.join([
				'Target: {}'.format(self.getTarget().getLabel()),
				'Tracked: {}'.format(self.getTarget().isTracked()),
				'Follow: {}'.format(self.isFollowingTarget()),
				'Shift: {}'.format(self.__shift),
				'Mimic: {}'.format(self.isMimingTarget()),
				'Offboard: {}'.format(self.isUsingOffboardControl()),
				'Mask: {}'.format(self.getMask())
				])
	
	#
	# Public methods to get and set parameters.
	#
	
	def followTarget(self, follow):
		"""Makes the drone follow or not its target.
		
		Args:
			follow (bool): Follow target if True.
		"""
		self.__follow = follow
	
	def getAttitude(self):
		"""Returns attitude replacement to control (DroneAttitude).
		
		The attitude is composed of roll, pitch, yaw, thrust.
		
		Depends on the mask and if the mimic is activated.
		
		The attitude can either be the manual attitude given with
		setManualAttitude or the target attitude given with setTarget.
		"""
		return self.__getAttitude()
	
	def getControlInput(self):
		"""Returns the last control input sent (DroneAttitude).
		
		The control input is an attitude (roll, pitch, yaw, thrust).
		
		Notes:
			- Is zero if no offboard control is performed.
			- Does not compute anything, just displays.
		"""
		return self.__controlInput
		
	def getError(self):
		"""Returns the last error (setpoint - pose) (DroneAttitude).
		
		The error is composed of X, Y, Yaw, Z (X and Y are
		projected on the drone body frame).
		"""
		return self.__error
		
	def getManualAttitude(self):
		"""Returns the last attitude entered by user (DroneAttitude).
		
		Replaces controller output if mask set as True.
		"""
		return self.__attitude
		
	def getManualSetpoint(self):
		"""Returns the last setpoint assigned manually (DronePose).
		
		A setpoint is composed of X, Y, Z and Yaw coordinates only.
		"""
		return self.__setpoint
		
	def getMask(self):
		"""Returns the control mask (bool[4]).
		
		Order: roll, pitch, yaw, thrust
		
		True = Replace controller output field by manual attitude.
		False = Use controller output.
		"""
		return self.__mask
		
	def getMeasurements(self):
		"""Returns the controller outputs (str).
		
		Structure: (attitudes)
			error
			total controlInput
			p-term
			i-term
			d-term
		
		See DroneAttitude for the exact structure of each term.
		"""
		return self.__getMeasurements()
		
	def getSeparatedOutputs(self):
		"""Returns separated values of the PIDs (DroneAttitude[3]).
		
		0 = P-term, 1 = I-term, 2 = D-term.
		"""
		return self.__separatedOutputs
		
	def getSetpoint(self):
		"""Returns the drone's current setpoint (DronePose).
		
		A setpoint is composed of X, Y, Z and Yaw coordinates only.
		
		Depends on the follow parameter.
		
		The setpoint can either be the manual setpoint given with
		setManualSetpoint or the target pose given with setTarget.
		"""
		return self.__getSetpoint()
		
	def getTarget(self):
		"""Returns the current target of the drone (RigidBody).
		
		See RigidBody to extract information about target.
		"""
		return self.__target
		
	def isFollowingTarget(self):
		"""Returns True if the drone is following its target.
		
		If False, the drone is reaching the manual setpoint.
		"""
		return self.__follow
		
	def isMimingTarget(self):
		"""Returns True if attitude control based on the target one.
		
		If False, the drone will try to reach the manual attitude.
		
		Notes:
			Will only have an effect on masked attitude fields.
		"""
		return self.__mimic
		
	def isUsingOffboardControl(self):
		"""Returns True if the controller is used for position control.
		
		If False, onboard PX4 controller will be used.
		"""
		return self.__offboard
		
	def mimicTarget(self, mimic):
		"""Sends the target attitude to the drone.
		
		The drone "mimics" the target (reaches the same attitude) if the
		correponding fields are masked.
		
		Args:
			mimic (bool): Sends target attitude if True.
		"""
		self.__mimic = mimic
		
	def setManualAttitude(self, roll, pitch, yaw, thrust):
		"""Assigns manually an attitude to the drone.
		
		Args:
			roll, pitch, yaw: Orientation of the drone [rad]
			thrust: Thrust of the drone [0-1]
			
		Note:
			- Disabled if not using WS controller.
			- Needs to be armed with OFFBOARD flight mode.
			- Will have an effect only if the corresponding component is
				set to True in the mask.
		"""
		self.__attitude = DroneAttitude(roll, pitch, yaw, thrust)
		
	def setManualSetpoint(self, x, y, z, yaw):
		"""Assigns a new setpoint to the drone.
		
		Args:
			x, y, z: Position to reach [m].
			yaw: Yaw to hover with [rad].
		
		Note:
			- Needs to be armed with OFFBOARD flight mode.
			- Will be reached only if not following target.
		"""
		self.__setpoint = DronePose(x, y, z, 0, 0, yaw)
		
	def setMask(self, roll, pitch, yaw, thrust):
		"""Enable or disable component of the drone attitude control.
		
		True = Replace controller output field by custom attitude.
		False = Use controller output.
		
		Args:
			roll, pitch, yaw, thrust (bool): True to replace component.
		"""
		self.__mask = [roll, pitch, yaw, thrust]
		
	def setTarget(self, label):
		"""Assigns a target to the drone from its VRPN label (str).
		
		The target is a rigid body streamed in VRPN that the drone
		will have to follow with a position shift defined in config.
		
		Note:
			- Needs to be armed with OFFBOARD flight mode.
			- Will be reached only if following target.
			- Will be imitated only if is using target attitude.
		"""
		self.__target = RigidBody(label)
		
	def useOffboardControl(self, offboard):
		"""Defines if the position control is done offboard.
		
		Uses PX4 onboard controller if False.
		
		Args:
			offboard (bool): Use offboard position control if True.
		"""
		self.__offboard = offboard
		
	#
	# Protected methods to allow controller to fill input values.
	#
	
	def _setControlInput(self, attitude):
		"""Fills control input field (numpy.array[4])."""
		self.__controlInput = DroneAttitude(*attitude)
		
	def _setError(self, error):
		"""Fills error field (numpy.array[4])."""
		self.__error = DroneAttitude(*error)
		
	def _setSetparatedOutputs(self, p, i, d):
		"""Fills the 3 separated PIDs outputs (3*numpy.array[4])."""
		self.__separatedOutputs = [	DroneAttitude(*p),
									DroneAttitude(*i),
									DroneAttitude(*d)]
	
	#
	# Private methods to make some computation.
	#
	
	def __getAttitude(self):
		"""Returns the attitude to reach (DroneAttitude)."""
		if self.__mimic:
			x, y, z, roll, pitch, yaw = self.__target.getPose()
			return DroneAttitude(roll, pitch, yaw, z)
		else:
			return self.__attitude
	
	def __getMeasurements(self):
		"""Creates the measurement string (str)."""		
		error = self.getError().toString('Error')
		cmd = self.getControlInput().toString('Total')
		
		separated = self.getSeparatedOutputs()
		p = separated[0].toString('P')
		i = separated[1].toString('I')
		d = separated[2].toString('D')
		
		return '\n\n'.join([error, cmd, p, i, d])
	
	def __getSetpoint(self):
		"""Returns the setpoint to reach (DronePose)."""
		if self.__follow:
			setpoint = self.__target.getPose().toArray()
			setpoint += self.__shift
			return DronePose(*setpoint)
		else:
			return self.__setpoint
