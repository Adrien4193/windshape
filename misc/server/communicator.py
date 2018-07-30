import socket
import os
import MySQLdb
import threading
import time

import dbapi
from Module import Module


# THREAD 1: Get modules state from NUCLEO
########################################################################
def recvUDP():

	time.sleep(0.1)
	
	print "[RECVTH] Receiving thread started..."

	# While sending thread is running
	while sendth.isAlive():

		# Receive a message on socket
		recvd, sAddr = sock.recvfrom(256)
		
		# Safety
		if len(recvd) == 0:
			continue
		
		# DATA (Module state)
		if recvd[0] == 'D':
			
			# Get data of interest from the incomming message
			data = recvd[2:recvd.index('\0')]
			isPowered, isFlashing, modID, _, rpm = data.split(';')
			ipAddr = sAddr[0]
			
			#print 'RECEIVED: '+str(recvd)+'\nFROM: '+str(ipAddr)  
			
			# Get sender
			if switch_fake:
				modID = int(modID)
			else:
				
				if ipAddr in id_from_ip.keys():
					modID = id_from_ip[ipAddr]
				else:
					continue
			
			# Safety
			if not modID in modules.keys():
				continue
			
			# Write immediately if new
			modules[modID].setAttribute('isPowered', isPowered)
			modules[modID].setAttribute('isFlashing', isFlashing)
			
			# Will be written at update
			modules[modID]['rpm'] = rpm
		
		# MAC
		elif recvd[0] == 'M':
			macAddr = recvd[2:-1]
			ipAddr = sAddr[0]
			
			#print 'RECEIVED: '+str(recvd)+'\nFROM: '+str(ipAddr)
			
			# Get ID from MAC
			modID = id_from_mac[macAddr]
			
			# Update IP
			if ipAddr in id_from_ip:
				del id_from_ip[ipAddr]
				id_from_ip[ipAddr] = modID
			
			# Write immediately if new
			modules[modID].setAttribute('ipAddr', ipAddr)
			modules[modID].resetLifePoints()

	print "[RECVTH] terminated"


# THREAD 2: Send modules state from DB to NUCLEO
def sendUDP():
	
	dbCon, dbCur = openDB_function()
	time.sleep(0.1)
	
	print "[SENDTH] Sending thread started..."
	
	# Sending rate
	dt = 0.01

	# While receiving thread is running
	while recvth.isAlive():
		
		for i in range(10):
			
			for mod in dbapi.getModules(dbCon, dbCur):
			
				isPowered = mod['isPowered']
				isFlashing = mod['isFlashing']
				pwm_str = mod['pwm']

				cmd = "O:{};{};{};{};{}\0".format(isPowered, isFlashing,
													0, 0, pwm_str)
				
				ipAddr = mod['ipAddr']
				
				try:
					sock.sendto(cmd, (ipAddr, port))
				except:
					print 'ERROR: CANNOT SEND '+str(cmd)+' TO '+str(ipAddr)

			time.sleep(dt)
		
		# Send broadcast message
		sock.sendto("B", ('<broadcast>', port))

		# Decay life points of modules
		for module in modules.values():
			module.decayLifePoints()

	print "[SENDTH] terminated"
	

# THREAD 3: Update RPM in DB
def updateDB():
	
	con, cur = openDB_function()
	time.sleep(0.1)
	
	print '[UPDATETH] Start ...'
	
	while recvth.isAlive():
	
		ref = time.time()
		
		cmd = 'UPDATE modules SET rpm=CASE modID '
		
		cmds = []
		values = []
		
		for modID in modules.keys():
			cmds.append('WHEN %s THEN %s')
			values.append(modID)
			values.append(modules[modID]['rpm'])
			
		cmds.append('ELSE rpm END;')
		
		cmd += ' '.join(cmds)
		
		with con:
			cur.execute(cmd, values)
		
		duration = time.time() - ref
		
		#print duration
		
		if duration < 1.0 / Module.rate:
			time.sleep(1.0/Module.rate - duration)
	
	print "[UPDATETH] terminated"
	
def create_modules(modulesDict):
	global id_from_ip, id_from_mac
	
	# Indexing IDs to find modules from IP or MAC
	id_from_ip = {}
	id_from_mac = {}
	
	con, cur = openDB_function()
	
	for modID in modulesDict.keys():
		
		macAddr = modulesDict[modID]['macAddr']
		x = modulesDict[modID]['posX']
		y = modulesDict[modID]['posY']
		
		# Index modules Ids with MAC for optimization
		id_from_mac[macAddr] = modID
		
		# Create Module instance
		modules[modID] = Module(con, cur, modID, macAddr, x, y)
	
def create_socket(port_number):
	global sock, port
	
	# Same as fake and modules
	port = port_number

	# Set socket as UDP and use IPv4
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

	# Make the socket reusable in case of inproper closure
	sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
	
	# Block until data available
	sock.setblocking(True)
	
	# Bind to port
	sock.bind(('', port))
	
	# Allow broadcast
	sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
	
# INITIALIZATION
########################################################################

def start(open_function, modulesDict, fake, port_number):
	
	global modules
	modules = {}
	
	global openDB_function
	openDB_function = open_function
	
	global switch_fake
	switch_fake = fake
	
	global recvth, sendth
	
	global log_file
	log_file = open('log.txt', 'w')
	
	# Create empty module table on DB
	dbCon, dbCur = open_function()
	dbapi.createModulesTable(dbCon, dbCur)
	dbapi.removeModuleFromDB(dbCon, dbCur, None)
	
	# Fill table with modules
	for modID in modulesDict.keys():
		dbapi.insertModuleToDB(dbCon, dbCur, modID, modulesDict[modID])
	
	# Communication with DB
	create_modules(modulesDict)
	
	# Init communication with modules
	create_socket(port_number)

	# Initiate and start a thread for receiving over UDP
	recvth = threading.Thread(target=recvUDP)
	recvth.setDaemon(True)
	recvth.start()

	# Initiate and start a thread for sending over UDP
	sendth = threading.Thread(target=sendUDP)
	sendth.setDaemon(True)
	sendth.start()
	
	# Update modules state in DB
	updateth = threading.Thread(target=updateDB)
	updateth.setDaemon(True)
	updateth.start()


