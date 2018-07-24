import weakref

# ROS main library
import rospy

# Drone control interface
from drone.Drone import Drone
# Fans array control interface
from fansarray.FansArray import FansArray
# Record information in files
from log.Recorder import Recorder


class Commander(object):
	"""Monitors drone and fans array.
	
	Inherits from: object.
	
	Overrides: __init__, __del__.
	"""
	
	def __init__(self):
		"""Creates drone and fans array managers."""
		rospy.logdebug('Commander initialization')
		
		# Attributes
		self.__drone = Drone()
		self.__fansArray = FansArray()
		self.__recorder = Recorder()
		
		# Drone status checks
		self.__tracked = False
		self.__targetTracked = False
		self.__connected = False
		self.__armed = False
		self.__mode = ''
		
		# Wind from drone pose
		self.__autoWind = rospy.get_param('~fansarray/auto_wind')
		
		# Real time update
		self.__timers = []
		self.__initTimers()
		
		rospy.logdebug('Commander inititalized')

	def __del__(self):
		"""Stops timers."""
		rospy.logdebug('Commander destruction')
		Commander.__close(weakref.ref(self))
		
	# ATTRIBUTES
	####################################################################
		
	def getDrone(self):
		"""Returns the drone interface (see Drone)."""
		return self.__drone
		
	def getFansArray(self):
		"""Returns the fans array interface (see FansArray)."""
		return self.__fansArray
		
	# COMMANDS
	####################################################################
	
	def setAutoWind(self, activate):
		"""Activate or desactivate automatic wind.
		
		Args:
			activate (bool): Activate auto wind if True.
		"""
		self.__autoWind = activate
		
	# STATUS REPORT
	####################################################################
	
	def getCommand(self):
		"""Returns command and error from drone controller (str)."""
		control = self.__drone.getControlParameters()
		
		error = control.getError().toString('Error')
		cmd = control.getControlInput().toString('Total')
		
		separated = control.getSeparatedOutputs()
		p = separated[0].toString('P')
		i = separated[1].toString('I')
		d = separated[2].toString('D')
		
		return '\n\n'.join([error, cmd, p, i, d])
		
	def getState(self, shift=0):
		"""Returns the drone pose and setpoint (str)."""
		control = self.__drone.getControlParameters()
		
		setpoint = control.getSetpoint().toString('Setpoint')
		mocap = self.__drone.getMocapPose().toString('Mocap')
		estimate = self.__drone.getEstimatedPose().toString('Estimate')
		
		return '\n\n'.join([setpoint, mocap, estimate])
	
	def getStatus(self, shift=0):
		"""Returns the drone and fans array status (str)."""
		return str(self.__drone)+'\n\n'+str(self.__fansArray)
		
	# REAL-TIME UPDATES
	####################################################################
	
	def __initTimers(self):
		"""Starts ROS timers."""
		# IMPORTANT: (will block destructor otherwise)
		# Create weakref on instance
		com = weakref.ref(self)
		
		# Updates drone status at defined rate
		rate = rospy.get_param('~update/update_rate')
		self.__startTimer(rate, lambda t: Commander.__update(com))
		
		# Records drone status, pose and control input at defined rate
		rate = rospy.get_param('~update/record_rate')
		self.__startTimer(rate, lambda t: Commander.__record(com))
		
		# Safety
		rospy.on_shutdown(lambda: Commander.__close(com))
		
	def __startTimer(self, rate, callback):
		"""Starts a ROS timer to call callback at rate."""
		period = 1.0 / rate
		timer = rospy.Timer(rospy.Duration(period), callback)
		self.__timers.append(timer)
	
	@staticmethod
	def __close(com):
		"""Closes all timers of commander (weakref)."""
		if com() is not None:
			for timer in com().__timers:
				timer.shutdown()
			com().__timers = []
	
	@staticmethod	
	def __record(com):
		"""Records drone status, pose and control input (weakref)."""
		if com() is not None:
			com().__recorder.record('status', com().getStatus())
			com().__recorder.record('pose', com().getState())
			com().__recorder.record('command', com().getCommand())
	
	@staticmethod	
	def __update(com):
		"""Checks drone state and generates wind (weakref)."""
		if com() is not None:
			com().__checkState()
			if com().__autoWind:
				com().__sendPWM()
		
	def __checkState(self):
		"""Checks if drone state has changed."""
		drone = self.__drone
		target = drone.getControlParameters().getTarget()
		
		if self.__tracked != drone.isTracked():
			self.__onTrackChange()
				
		if self.__targetTracked != target.isTracked():
			self.__onTargetTrackingChange()
				
		if self.__connected != drone.isConnected():
			self.__onConnectionChange()
				
		if self.__armed != drone.isArmed():
			self.__onArmingChange()
		
		if self.__mode != drone.getFlightMode():
			self.__onModeChange()
	
	def __sendPWM(self):
		"""Sends a PWM value corresponding to drone pose."""
		if not self.__drone.isTracked():
			return
			
		if not self.__fansArray.isPowered():
			return
		
		x = self.__drone.getMocapPose().getX()
		pwm = int(50 * x)
		
		if pwm > 60:
			pwm = 60
		elif pwm < 5:
			pwm = 5
		
		self.__fansArray.setPWM(pwm)
	
	# ON CHANGE METHODS
	####################################################################
	
	def __onTrackChange(self):
		"""Action to perform if drone tracking has changed."""
		self.__tracked = not self.__tracked
		
		if self.__tracked:
			rospy.loginfo('Drone tracked')
		else:
			rospy.logwarn('Drone tracking lost')
		
	def __onTargetTrackingChange(self):
		"""Action to perform if target has changed."""
		self.__targetTracked = not self.__targetTracked
		
		if self.__targetTracked:
			rospy.loginfo('Target tracked')
		else:
			rospy.logwarn('Target tracking lost')
		
	def __onConnectionChange(self):
		"""Action to perform if drone connection has changed."""
		self.__connected = not self.__connected
		
		if self.__connected:
			rospy.loginfo('Drone connected')
		else:
			rospy.logwarn('Drone disconnected')
		
	def __onArmingChange(self):
		"""Action to perform if drone armed or disarmed."""
		self.__armed = not self.__armed
		
		if self.__armed:
			rospy.loginfo('Drone armed')
		else:
			rospy.loginfo('Drone disarmed')
		
	def __onModeChange(self):
		"""Action to perform if drone flight mode has changed."""
		self.__mode = self.__drone.getFlightMode()
		rospy.loginfo('New flight mode: %s', self.__mode)
