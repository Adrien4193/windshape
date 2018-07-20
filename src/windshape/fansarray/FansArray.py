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
	
	# INITIALIZER AND DESTRUCTOR
	####################################################################
	
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
		self.__close()
		
	def __close(self):
		"""Set PWM to 0 and turn off PSU."""
		self.setPWM(0)
		self.turnOnPSU(False)
		
		# Force DB update to be sure
		con, cur = self.__openDB()
		self.__updateAttribute(con, cur, 'pwm')
		self.__updateAttribute(con, cur, 'isPowered')
		
	# ATTRIBUTES
	####################################################################
	
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
		
	# COMMANDS
	####################################################################
		
	def setPWM(self, pwm):
		"""Assigns a PWM value to all modules."""
		rospy.logdebug('Set pwm to: %d', int(pwm))

		for module in self.__modules.values():
			pwm_str = ','.join(len(self.__modules)*[str(int(pwm))])
			module.setAttribute('pwm', pwm_str)

	def turnOnPSU(self, powered):
		"""Turns on Power Supply Units if powered is True."""
		rospy.loginfo('Turn on PSU: '+str(powered))
		
		for module in self.__modules.values():
			module.setAttribute('isPowered', int(powered))
			
	# STATUS AS STRING
	####################################################################
			
	def __str__(self):
		"""Returns a summary of the fans array status."""
		string = [	'Host: {}'.format(self.__serverIP),
					'Powered: {}'.format(self.isPowered()),
					'PWM: {}'.format(self.getPWM())]
		
		return '\n'.join(string)
		
	# MAIN LOOP
	####################################################################
	
	def run(self):
		"""Updates the database at given rate."""
		
		# Wait for server connection
		self.__loadModules()
		
		# Cursors for this thread
		con, cur = self.__openDB()
		
		# Starts update
		rate = rospy.Rate(rospy.get_param('~fansarray/db_update_rate'))
		while not rospy.is_shutdown():
			self.__updateDB(con, cur)
			rate.sleep()
		
	def __updateDB(self, con, cur):
		"""Updates the DB from the stored ones.
		
		Query structure:
		
			UPDATE modules SET pwm=CASE modID
			
			WHEN <id 1> THEN <pwm 1>
			...
			
			WHEN <id n> THEN <pwm n>
			
			ELSE pwm END;
		"""
		if not self.__modules:
			return
		
		# Updates modules attributes if needed
		for module in self.__modules.values():
			for attribute in module.keys():
				if module.updateAttribute(attribute):
					self.__updateAttribute(con, cur, attribute)
			
	def __updateAttribute(self, con, cur, attribute):
		"""Updates an attribute of the modules."""
		
		# Updates PWM field as a function of modID field
		cmd = 'UPDATE modules SET '+attribute+'=CASE modID '
		
		# commandes: WHEN modID THEN pwm
		cmds = []
		values = []
		
		for modID, module in self.__modules.items():
			cmds.append('WHEN %s THEN %s')
			values.append(modID)
			values.append(module[attribute])
		
		# If not in list: Don't touch !
		cmds.append('ELSE pwm END;')
		
		cmd += ' '.join(cmds)
		
		# Executes SQL command
		with con:
			cur.execute(cmd, values)
			
	# DB CONNECTION
	####################################################################
	
	def __loadModules(self):
		"""Fetches modules in DB and stores them as Module instances."""
		rospy.loginfo('Loading modules from database')
		
		# Try to find DB host
		self.__findServer()
		
		# Loads modules from DB
		con, cur = self.__openDB()	
		self.__getModulesFromDB(con, cur)
		
		rospy.loginfo('Modules successfully loaded')
	
	def __findServer(self):
		"""Returns DB host IP (str) if server is broadcasting.
		
		Notes:
			Blocks until message received.
		"""
		port = rospy.get_param('~fansarray/broadcast_port')
		rospy.loginfo('Waiting for server broadcast on %d', port)
		
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
		
	def __openDB(self):
		"""Open a MySQL connection at given IP (str).
		
		Returns a MySQLConnection and a MySQLCursor (two objects).
		"""
		if self.__serverIP == None:
			rospy.logerr('No server to open DB')

		con = MySQLdb.connect(	host=self.__serverIP,
								user="ws_user",
								passwd="Aero-1234",
								db="mysql")
		cur = con.cursor()
		
		return con, cur
		
	def __getModulesFromDB(self, con, cur):
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
