import socket
import MySQLdb
import threading
import random

# ROS main library
import rospy

# Module representation
from Module import Module


class FansArray(threading.Thread):
	"""Communicates with the fans array.
	
	Communicates with the fans array server hosting a database. Uses
	MySQL requests to write and read into the database. The server
	reads the DB in loop and sends the PWM values to the fans.
	
	Module = 2 layers of 9 fans = 18 fans.
	
	Written in Module: "pwm" field: "PWM0, PWM1, ..., PWM17".
	Can also power on or off a Module using "isPowered" field.
	
	Other fields (RPM, isConnected, ...) are read from DB regularly
	but not written.
	
	Main loop = write commands in DB and read attributes from DB.
	
	Inherits from: threading.Thread.
	
	Overrides: __init__, __del__, __str__, run.
	"""
	
	def __init__(self):
		"""Creates modules instances and auto start."""
		rospy.logdebug('FansArray initialization')
		
		super(FansArray, self).__init__()
		
		# Database server IP (auto-discovered)
		self.__serverIP = None
		
		# Modules
		self.__modules = {}
		
		# Wind function
		self.__windFunction = None
		self.__ref = rospy.get_time()
		
		# Run main loop
		self.setDaemon(True)
		self.start()
		
		rospy.logdebug('FansArray started')
		
	def __del__(self):
		"""Set PWM to 0 and switch off the power supply."""
		rospy.logdebug('FansArray destruction')
		
		if self.__serverIP is None:
			return
		
		con, cur = self.__openDB()
		self.setPWM(0)
		self.turnOnPSU(False)
		self.__writeToDB(con, cur, 'pwm')
		self.__writeToDB(con, cur, 'isPowered')
	
	def __str__(self):
		"""Returns a summary of the fans array status."""
		return '\n'.join([
					'Host: {}'.format(self.__serverIP),
					'Modules: {}'.format(len(self.__modules)),
					'Connected: {}'.format(self.isConnected()),
					'Powered: {}'.format(self.isPowered())
					])
	
	#
	# Public methods to get status and send commands.
	#
	
	def getPWM(self):
		"""Returns the current PWM values (str)."""
		pwms = []
		
		for module in self.__modules.values():
			pwms.append(str(module['pwm']))
		
		return '\n'.join(pwms)
		
	def isConnected(self):
		"""Returns True if connected to database."""
		return self.__serverIP is not None
		
	def isPowered(self):
		"""Returns True if the power supply is on."""
		if self.__modules:
			modID = self.__modules.keys()[0]  # Takes a random module
			return bool(self.__modules[modID]['isPowered'])
		
		return False
		
	def setPWM(self, pwm):
		"""Set a PWM value (int) to all fans."""
		self.__windFunction = None
		pwm = self.__validatePWM(pwm)
		
		for module in self.__modules.values():
			pwm_str = ','.join(len(self.__modules)*[str(int(pwm))])
			module.setAttribute('pwm', pwm_str)
	
	def setWindFunction(self, windFunction):
		"""Run a wind function on the fans (str).
		
		Variables are x, y and t.
		"""
		self.__windFunction = windFunction
		self.__ref = rospy.get_time()
		
	def __runWindFunction(self):
		"""Evaluates wind function and sends PWM values."""
		if self.__windFunction is None:
			return
		
		windFunction = self.__windFunction
		
		# Time
		t = rospy.get_time() - self.__ref
		
		for module in self.__modules.values():
			cmds = []
			
			# Module info
			modID = int(module['modID'])
			posX = int(module['posX'])
			posY = int(module['posY'])
			
			# Get pwm of each fan from XY position (2*9 commands)
			for j in range(3):
				for i in range(3):
					
					# XY position
					x = (3*(posX-1) + i) * 0.08
					y = (3*(posY-1) + j) * 0.08
					
					# Evaluates wind function
					try:
						pwm = int(eval(str(self.__windFunction)))
					except Exception as e:
						rospy.logerr(e)
						return
					
					pwm = self.__validatePWM(pwm, 0, 50)
					
					# Add current fans (front + rear) to module command
					cmds.append(str(int(pwm)))
					cmds.append(str(int(pwm)))
			
			# Store in modules dict
			pwm_str = ','.join(cmds)
			module.setAttribute('pwm', pwm_str)
	
	def turnOnPSU(self, powered):
		"""Turns on Power Supply Units if powered is True."""
		rospy.loginfo('Turn on PSU: '+str(powered))
		for module in self.__modules.values():
			module.setAttribute('isPowered', int(powered))
	
	#
	# Private methods
	#
	
	def run(self):
		"""Updates the database at given rate."""
		rate = rospy.Rate(rospy.get_param('~fansarray/db_update_rate'))
		
		while not rospy.is_shutdown():
		
			# Wait for server connection
			con, cur = self.__connectToDB()
			
			# Update DB in loop
			while not rospy.is_shutdown():
				try:
					self.__runWindFunction()
					self.__updateDB(con, cur)
				except MySQLdb.Error as e:
					rospy.logerr(e)
					self.__serverIP = None
					self.__modules = {}
					break
				rate.sleep()
		
	def __findServer(self):
		"""Returns DB host IP (str) if server is broadcasting.
		
		Notes:
			Blocks until message received.
		"""
		port = rospy.get_param('~fansarray/broadcast_port')
		rospy.loginfo('Waiting for server broadcast on port %d', port)
		
		# Client socket on known port
		sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
		sock.setblocking(True)
		sock.bind(('', port))
		
		while not rospy.is_shutdown():
			
			# Listens for broadcasts from database host
			message, sAddr = sock.recvfrom(256)
			rospy.logdebug('Message received: %s', message)
			
			# Checks message received is the server broadcast
			if message == 'WINDSHAPE':
				rospy.loginfo('Server found at %s', sAddr[0])
				self.__serverIP = sAddr[0]
				break
		
	def __loadModules(self, con, cur):
		"""Get modules list from database."""
		# Fetches modules (list of tuples)
		with con:
			cur.execute("""
						SELECT modID, macAddr, ipAddr, posX, posY,
								nFans, nLayers, pwm, rpm, 
								isConnected, isFlashing, isPowered,
								lifePoints, isRebooting, isSendingRPM 
						FROM  modules
						""")
		modules = cur.fetchall()
		
		# Stores modules
		for module in modules:
			modID = module[0]
			self.__modules[modID] = Module(module)
			
	def __connectToDB(self):
		"""Fetches modules in DB and stores them as Module instances."""
		rospy.loginfo('Loading modules from database')
		
		# Block until DB host found
		self.__findServer()
		
		# Loads modules from DB
		con, cur = self.__openDB()
		self.__loadModules(con, cur)
		
		rospy.loginfo('Modules successfully loaded')
		
		return con, cur
			
	def __openDB(self):
		"""Open a MySQL connection at given IP (str).
		
		Returns a MySQLConnection and a MySQLCursor (two objects).
		"""
		con = MySQLdb.connect(	host=self.__serverIP,
								user="ws_user",
								passwd="Aero-1234",
								db="mysql")
		cur = con.cursor()
		
		return con, cur
		
	def __readFromDB(self, con, cur, attribute):
		"""Reads an attribute from DB and updates modules."""
		# Fetches attribute for all modules with modID
		with con:
			cur.execute('SELECT modID, '+attribute+' FROM modules')
		
		# 2 values per module (0=modID, 1=value of attribute)
		modules = cur.fetchall()
		
		# Updates attribute for all modules
		for module in modules:
			modID = module[0]
			self.__modules[modID][attribute] = module[1]
		
	def __updateDB(self, con, cur):
		"""Updates the DB and the modules in memory."""
		if self.__modules:
			module = self.__modules.values()[0]  # Takes random module
			
			for attribute in module.keys():
				if module.isPermitted(attribute):
					self.__writeToDB(con, cur, attribute)
				else:
					self.__readFromDB(con, cur, attribute)
	
	def __validatePWM(self, pwm, min_=0, max_=100):
		"""Resets a PWM value as an integer between 0 and 100."""
		pwm = int(pwm)
		
		# Manual limits
		if pwm > max_:
			pwm = int(max_)
		elif pwm < min_:
			pwm = int(min_)
		
		# Physical limits
		if pwm < 5:
			pwm = 0
		elif pwm > 100:
			pwm = 100
		
		return pwm
		
	def __writeToDB(self, con, cur, attribute):
		"""Updates an attribute of the modules.
		
		Query structure:
		
			UPDATE modules SET pwm=CASE modID
			
			WHEN <id 1> THEN <pwm 1>
			...
			
			WHEN <id n> THEN <pwm n>
			
			ELSE pwm END;
		"""
		# Updates attribute as a function of modID
		cmd = 'UPDATE modules SET '+attribute+'=CASE modID '
		
		cmds = []
		values = []
		
		# commands: WHEN modID THEN value
		for modID, module in self.__modules.items():
			if module.needsUpdate(attribute):
				cmds.append('WHEN %s THEN %s')
				values.append(modID)
				values.append(module[attribute])
		
		# If not in list: Don't touch !
		cmds.append('ELSE '+attribute+' END;')
		
		# Final query
		cmd += ' '.join(cmds)
		
		# Executes SQL query only if needed
		if values:
			with con:
				cur.execute(cmd, values)
