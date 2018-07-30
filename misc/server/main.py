import socket
import time
import atexit
import random
import MySQLdb

import communicator
import configurator
import dbapi

def exitFunction():
	print "Bye!"

print "\nCOMMUNICATOR PROGRAM STARTED\n"
print "version 3.0"

atexit.register(exitFunction)

def openDB_function():
	
	# Connect to local MySQL database
	con = MySQLdb.connect(host="localhost",    # your host, usually localhost
	                 	  user="root",         # your username
	                      passwd="Aero-1234",  # your password
	                      db="mysql")
	cur = con.cursor()
	return con, cur

# PARAMETERS
########################################################################

# Simulate modules communication if set
switch_fake = False

# Sending rate (fake modules)
rate = 100.0 # [Hz]

# Choose database host
#openDB_function = dbapi.openDB_TerenceHill
#openDB_function = dbapi.openDB_localhostOSX
#openDB_function = dbapi.openDB_wsServer

# Communication port with modules
port = 58083

# Modules configuration file
configFile = "configurations/modules.conf"
fanNumbers=9
fanLayers=2

########################################################################

# Read the config file and generate a modules dictionnary
modulesDict = configurator.getModulesStructure(configFile, fanNumbers, fanLayers)

# Initialize communicator
communicator.start(openDB_function, modulesDict, switch_fake, port)

# Create fake socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Internet, no connection
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Adress
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1) # Port
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) # Broadcast

# MAIN LOOP
while communicator.recvth.isAlive():
	
	# Allows to turn off the FAKE STUFF
	if switch_fake:
	
		# SEND DATA 10 times at given rate
		for i in range(1):
			
			# DATA
			for modID in modulesDict.keys():
				
				header = 'D'
				isPowered = '1'
				isFlashing = '1'
				
				# FAKE (include modID in message to get sender at reception)
				name = str(modID)
				
				unused = '1'
				
				rpms = []
				
				for i in range(18):
					rpms.append(str(int(10000*random.random())))
				
				data = ','.join(rpms)

				message = header+':' +';'.join([isPowered, isFlashing,
											name, unused, data]) + '\0'
				
				#print 'SEND: '+message
				
				s.sendto(message, ('localhost', port))
			
			time.sleep(1.0/rate)
			
		# SEND MAC 1 time
		for modID in modulesDict.keys():
			
			# MAC
			header = 'M'
			macAddr = modulesDict[modID]['macAddr']
			
			message = header+':' + str(macAddr) + '\0'
			
			#print 'SEND: '+message
			
			s.sendto(message, ('localhost', port))
		
	else:
		time.sleep(1)
	
	s.sendto("WINDSHAPE", ('<broadcast>', 58084))
	
	

