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
		
	# STATUS
	####################################################################
	
	def getCommand(self):
		"""Returns command and error from drone controller (str)."""
		control = self.__drone.getControlParameters()
		
		command = 'control input:\n'+str(control.getControlInput())
		error = 'error:\n'+str(control.getError())
		
		command = command.replace('\n', '\n    ')
		error = error.replace('\n', '\n    ')
		
		return '\n\n'.join([command, error])
		
	def getState(self):
		"""Returns the drone pose and setpoint (str)."""
		setpoint = 'setpoint:\n'+str(self.__drone.getSetpoint())
		mocap = 'mocap:\n'+str(self.__drone.getMocapPose())
		estimate = 'estimate:\n'+str(self.__drone.getEstimatedPose())
		
		setpoint = setpoint.replace('\n', '\n    ')
		mocap = mocap.replace('\n', '\n    ')
		estimate = estimate.replace('\n', '\n    ')
		
		return '\n\n'.join([setpoint, mocap, estimate])
	
	def getStatus(self):
		"""Returns the drone and fans array status (str)."""
		return str(self.__drone)+'\n\n'+str(self.__fansArray)
		
	# TIMERS
	####################################################################
	
	def __initTimers(self):
		"""Starts ROS timers."""
		
		# IMPORTANT: (will block destructor otherwise)
		# Create weakref on instance
		commander = weakref.ref(self)
		
		# Updates drone status at defined rate
		rate = rospy.get_param('~update/update_rate')
		self.__startTimer(rate, lambda t: commander().__update())
		
		# Records drone status, pose and control input at defined rate
		rate = rospy.get_param('~update/record_rate')
		self.__startTimer(rate, lambda t: commander().__record())
		
		# Safety
		rospy.on_shutdown(lambda: commander().__close())
		
	def __startTimer(self, rate, callback):
		"""Starts a ROS timer to call callback at rate."""
		period = 1.0 / rate
		timer = rospy.Timer(rospy.Duration(period), callback)
		self.__timers.append(timer)
		
	def __close(self):
		"""Closes all timers."""
		for timer in self.__timers:
			timer.shutdown()
		
	# UPDATE METHODS
	####################################################################
		
	def __record(self):
		"""Records drone status, pose and control input."""
		self.__recorder.record('status', self.getStatus())
		self.__recorder.record('pose', self.getState())
		self.__recorder.record('command', self.getCommand())
		
	def __update(self):
		"""Checks drone state and updates fans array if tracked."""
		self.__checkState()
		
		if self.__autoWind:
			self.__sendPWM()
		
	def __checkState(self):
		"""Checks if drone state has changed."""
		drone = self.__drone
		target = drone.getTarget()
		
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
		self.__tracked = self.__drone.isTracked()
		
		if self.__tracked:
			rospy.loginfo('Drone tracked')
		else:
			rospy.logwarn('Drone tracking lost')
		
	def __onTargetTrackingChange(self):
		"""Action to perform if target has changed."""
		self.__targetTracked = self.__drone.getTarget().isTracked()
		
		if self.__targetTracked:
			rospy.loginfo('Target tracked')
		else:
			rospy.logwarn('Target tracking lost')
		
	def __onConnectionChange(self):
		"""Action to perform if drone connection has changed."""
		self.__connected = self.__drone.isConnected()
		
		if self.__connected:
			rospy.loginfo('Drone connected')
		else:
			rospy.logwarn('Drone disconnected')
		
	def __onArmingChange(self):
		"""Action to perform if drone armed or disarmed."""
		self.__armed = self.__drone.isArmed()
		
		if self.__armed:
			rospy.loginfo('Drone armed')
		else:
			rospy.loginfo('Drone disarmed')
		
	def __onModeChange(self):
		"""Action to perform if drone flight mode has changed."""
		self.__mode = self.__drone.getFlightMode()
		rospy.loginfo('New flight mode: %s', self.__mode)
