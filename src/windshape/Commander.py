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
	
	Uses a Drone and a FansArray instances (that can be accessed by
	user) to communicate with the real devices.
	
	See Drone and FansArray to communicate with them.
	
	Uses separated threads to record the drone status, pose and control
	inputs, warn user about changes and control the wind as a function
	of the drone pose.
	
	Inherits from: object.
	
	Overrides: __init__, __del__, __str__.
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
		
	def __str__(self):
		"""Describes the drone and fans array status.
		
		See Drone and FansArray __str__ for more details.
		"""
		return (str(self.__drone)+'\n\n'+str(self.__fansArray)+'\n'
				+'Auto-wind: '+str(self.__autoWind))
	
	#
	# Public methods to access drone and fans array interfaces and
	# activate the wind as a function of the drone pose.
	#
		
	def getDrone(self):
		"""Returns the drone interface (Drone).
		
		See Drone for usage.
		"""
		return self.__drone
		
	def getFansArray(self):
		"""Returns the fans array interface (FansArray).
		
		See FansArray for usage.
		"""
		return self.__fansArray
	
	def setAutoWind(self, activate):
		"""Activate or desactivate automatic wind.
		
		Auto-wind is proportional to the drone X position.
		
		Args:
			activate (bool): Activate auto wind if True.
		"""
		if not activate and self.__autoWind:
			self.__fansArray.setPWM(0)
		
		self.__autoWind = activate
	
	#	
	# Private methods for real-time update in separated threads.
	#
	
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
			rec = com().__recorder
			control = com().__drone.getControlParameters()
			rec.record('status', str(com()))
			rec.record('pose', com().__drone.getMeasurements())
			rec.record('command', control.getMeasurements())
	
	@staticmethod	
	def __update(com):
		"""Checks drone state and generates wind (weakref)."""
		if com() is not None:
			com().__checkState()
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
	
	def __initTimers(self):
		"""Starts ROS timers for real-time updates."""
		# IMPORTANT: (will block destructor otherwise)
		# Creates weakref on instance
		com = weakref.ref(self)
		
		# Updates drone status at defined rate
		rate = rospy.get_param('~update/update_rate')
		self.__startTimer(rate, lambda t: Commander.__update(com))
		
		# Records drone status, pose and control input at defined rate
		rate = rospy.get_param('~update/record_rate')
		self.__startTimer(rate, lambda t: Commander.__record(com))
		
		# Safety
		rospy.on_shutdown(lambda: Commander.__close(com))
		
	def __onArmingChange(self):
		"""Action to perform if drone armed or disarmed."""
		self.__armed = not self.__armed
		
		if self.__armed:
			rospy.loginfo('Drone armed')
		else:
			rospy.loginfo('Drone disarmed')
			
	def __onConnectionChange(self):
		"""Action to perform if drone connection has changed."""
		self.__connected = not self.__connected
		
		if self.__connected:
			rospy.loginfo('Drone connected')
		else:
			rospy.logwarn('Drone disconnected')
			
	def __onModeChange(self):
		"""Action to perform if drone flight mode has changed."""
		self.__mode = self.__drone.getFlightMode()
		rospy.loginfo('New flight mode: %s', self.__mode)
		
	def __onTargetTrackingChange(self):
		"""Action to perform if target has changed."""
		self.__targetTracked = not self.__targetTracked
		
		if self.__targetTracked:
			rospy.loginfo('Target tracked')
		else:
			rospy.logwarn('Target tracking lost')
	
	def __onTrackChange(self):
		"""Action to perform if drone tracking has changed."""
		self.__tracked = not self.__tracked
		
		if self.__tracked:
			rospy.loginfo('Drone tracked')
		else:
			rospy.logwarn('Drone tracking lost')
	
	def __sendPWM(self):
		"""Sends a PWM value from drone pose."""
		if not self.__fansArray.isPowered():
			return
		
		if not self.__autoWind:
			return
		
		if self.__drone.isTracked():
			gain = rospy.get_param('~control/pwm_gain')
			min_ = rospy.get_param('~control/min_wind')
			max_ = rospy.get_param('~control/max_wind')
			
			x = self.__drone.getPoseMocap().getX()
			pwm = int(gain * x)
			
			if pwm > max_:
				pwm = max_
			elif pwm < min_:
				pwm = min_
		else:
			pwm = 0
		
		self.__fansArray.setPWM(pwm)
		
	def __startTimer(self, rate, callback):
		"""Starts a ROS timer to call callback at rate."""
		period = 1.0 / rate
		timer = rospy.Timer(rospy.Duration(period), callback)
		self.__timers.append(timer)
