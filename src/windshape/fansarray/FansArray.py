import socket
import MySQLdb
import threading

# ROS main library
import rospy

# Module representation
from Module import Module


class FansArray(threading.Thread):
	"""Send PWM values to the fans array.
	
	Communicates with the fans array server that hosts a database. Uses
	MySQL requests to write into the database that the server reads in
	loop and sends the PWM values to the fans.
	
	Structure: Module = 2 layers of 9 fans = 18 fans.
	
	Written in Module: "pwm" field: "PWM1, PWM2, ..., PWM18"
	
	Can also power on or off a Module using "isPowered" field.
	
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
		
		# Run main loop
		self.setDaemon(True)
		self.start()
		
		rospy.logdebug('FansArray started')
		
	def __del__(self):
		"""Set PWM to 0 and switch off the power supply."""
		rospy.logdebug('FansArray destruction')
		
		self.setPWM(0)
		self.turnOnPSU(False)
		
		# Updates DB manually because thread is not running any more
		con, cur = self.__openDB()
		self.__updateAttribute(con, cur, 'pwm')
		self.__updateAttribute(con, cur, 'isPowered')
	
	def __str__(self):
		"""Returns a summary of the fans array status."""
		return '\n'.join([	'Host: {}'.format(self.__serverIP),
					'Modules {}'.format(len(self.__modules)),
					'Powered: {}'.format(self.isPowered()),
					'PWM: {}'.format(self.getPWM())
					])
	
	#
	# Public methods to get status and send commands.
	#
	
	def getPWM(self):
		"""Returns the current PWM value (int) of the fans."""
		if self.__modules:
			modID = self.__modules.keys()[0]  # Takes a random module
			string = self.__modules[modID]['pwm']
			
			return int(string.split(',', 2)[0])
		
		return 0
		
	def isPowered(self):
		"""Returns True if the power supply is on."""
		if self.__modules:
			modID = self.__modules.keys()[0]  # Takes a random module
			return bool(self.__modules[modID]['isPowered'])
		
		return False
		
	def setPWM(self, pwm):
		"""Assigns a PWM value to all modules."""
		for module in self.__modules.values():
			pwm_str = ','.join(len(self.__modules)*[str(int(pwm))])
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
					self.__updateDB(con, cur)
				except MySQLdb.Error as e:
					rospy.logerr(e)
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
		
		message, sAddr = sock.recvfrom(256)
		
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
			rospy.logdebug('Module loaded: %s', self.__modules[modID])
			
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
		if self.__serverIP == None:
			rospy.logfatal('No server to open DB')

		con = MySQLdb.connect(	host=self.__serverIP,
								user="ws_user",
								passwd="Aero-1234",
								db="mysql")
		cur = con.cursor()
		
		return con, cur
		
	def __updateAttribute(self, con, cur, attribute):
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
		
		# commandes: WHEN modID THEN value
		cmds = []
		values = []
		
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
			
	def __updateDB(self, con, cur):
		"""Updates the DB from the modules in memory."""
		if self.__modules:
			modID = self.__modules.keys()[0]  # Takes random module
			
			for attribute in self.__modules[modID].keys():
				self.__updateAttribute(con, cur, attribute)
			
