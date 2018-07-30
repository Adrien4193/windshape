import time

import MySQLdb

print "dbapi v3.0"


def openDB_TerenceHill():
	con = MySQLdb.connect(host='192.168.1.121', #host="192.168.88.249",    # your host, usually localhost
	                 	  user="windControl_user",         # your username
	                      passwd="Aero-1234",  # your password
	                      db="mysql")
	cur = con.cursor()
	return con, cur


def openDB_xampp():
	con = MySQLdb.connect(host='192.168.64.2', #host="192.168.88.249",    # your host, usually localhost
	                 	  user="root2",         # your username
	                      passwd="Aero-1234",  # your password
	                      db="mysql")
	cur = con.cursor()
	return con, cur


def openDB_wsServer():

	# Connect to local MySQL database
	con = MySQLdb.connect(host="192.168.1.122",    # your host, usually localhost
	                 	  user="ws_user",         # your username
	                      passwd="Aero-1234",  # your password
	                      db="mysql")
	cur = con.cursor()
	return con, cur


def closeDB(con):
	if con:
		con.close()

def createModulesTable(con, cur):
	""" Create a modules table in the given database. """
	with con:
		cur.execute("""CREATE TABLE IF NOT EXISTS modules (
			 	 modID int NOT NULL,
				 macAddr text,
				 ipAddr text,
				 posX int,
				 posY int,
				 nFans int,
				 nLayers int,
				 pwm text,
				 rpm text,
				 isConnected int,
				 isFlashing int,
				 isPowered int,
				 lifePoints int,
				 isRebooting int,
				 isSendingRPM int,
				 PRIMARY KEY (modID))
				 """)

def insertModuleToDB(con, cur, modID, modAttributesDict):
	md = modAttributesDict
	with con:
		cur.execute("""INSERT INTO modules 
			(modID, macAddr, ipAddr, posX, posY, nFans, nLayers, 
			 pwm, rpm, isConnected, isFlashing, isPowered, lifePoints, isRebooting, isSendingRPM) 
			 VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s);""", 
			 (modID, md['macAddr'], md['ipAddr'], md['posX'], md['posY'], md['nFans'], md['nLayers'], 
			  md['pwm'], md['rpm'], 0, 0, 0, 100, 0, 0))

def removeModuleFromDB(con, cur, modID):
	with con:
		if type(modID) is list:
			modIDs = modID
			for modID in modIDs:
				cur.execute("DELETE FROM modules WHERE name=%s", (modID,))
		elif type(modID) is int:
			cur.execute("DELETE FROM modules WHERE name=%s", (modID,))
		elif modID is None:
			cur.execute("DELETE FROM modules")

def setCustomField(con, cur, entryKey, entryValue, targetKey, targetValue):
	with con:
		cmd = "UPDATE modules SET "+str(entryKey)+"=%s"+" WHERE "+str(targetKey)+"=%s"
		cur.execute(cmd, [entryValue, targetValue])

def getCustomField(con, cur, entryKey, entryValue):
	with con:
		cmd =  " SELECT modID, macAddr, ipAddr, posX, posY, nFans, nLayers, pwm, rpm, isConnected, isFlashing, isPowered, lifePoints, isRebooting, isSendingRPM FROM modules WHERE "+str(entryKey)+"=%s "
		cur.execute(cmd, [entryValue,])
	return tuplist2dictlist(cur.fetchall())

def setIP(con, cur, ip, value, key='macAddr'):
	with con:
		if key == "macAddr":
			cur.execute("UPDATE modules SET ipAddr=%s WHERE macAddr=%s", (ip, value))
		elif key == 'modID':
			cur.execute("UPDATE modules SET ipAddr=%s WHERE modIO=%s", (ip, value))
	
def getModuleFromDB_id(con, cur, modID):
	with con:
		cur.execute(""" SELECT modID, macAddr, ipAddr, posX, posY, nFans, nLayers, pwm, rpm, 
			            isConnected, isFlashing, isPowered, lifePoints, isRebooting, isSendingRPM 
			            FROM modules WHERE modID = %s""", (modID,))
	return tup2dict(cur.fetchone())

def getModuleFromDB_ip(con, cur, ipAddr):
	with con:
		cur.execute(""" SELECT modID, macAddr, ipAddr, posX, posY, nFans, nLayers, pwm, rpm, 
			            isConnected, isFlashing, isPowered, lifePoints, isRebooting, isSendingRPM 
			            FROM modules WHERE ipAddr = %s""", (ipAddr,))
	return tup2dict(cur.fetchone())

def getModuleFromDB_mac(con, cur, macAddr):
	with con:
		cur.execute(""" SELECT modID, macAddr, ipAddr, posX, posY, nFans, nLayers, pwm, rpm, 
			            isConnected, isFlashing, isPowered, lifePoints, isRebooting, isSendingRPM 
			            FROM modules WHERE macAddr = %s""", (macAddr,))
	return tup2dict(cur.fetchone())

def getModulesConnected(con, cur):
	with con:
		cur.execute(""" SELECT modID, macAddr, ipAddr, posX, posY, nFans, nLayers, pwm, rpm, 
			            isConnected, isFlashing, isPowered, lifePoints, isRebooting, isSendingRPM 
			            FROM modules WHERE isConnected = 1 """)
	return tuplist2dictlist(cur.fetchall())

def getModules(con, cur):
	with con:
		cur.execute(""" SELECT modID, macAddr, ipAddr, posX, posY, nFans, nLayers, pwm, rpm, 
			            isConnected, isFlashing, isPowered, lifePoints, isRebooting, isSendingRPM 
			            FROM  modules """)
	return tuplist2dictlist(cur.fetchall())

def tuplist2dictlist(tuplist):
	dictlist = []
	for mod in tuplist:
		modDict = {'modID': mod[0],
				   'macAddr': mod[1],
				   'ipAddr': mod[2],
				   'posX': mod[3],
				   'posY': mod[4],
				   'nFans': mod[5],
			       'nLayers': mod[6],
			       'pwm': mod[7],
				   'rpm': mod[8],
				   'isConnected': mod[9],
				   'isFlashing': mod[10],
			       'isPowered': mod[11],
			       'lifePoints': mod[12],
		           'isRebooting': mod[13],
		           'isSendingRPM': mod[14]}
		dictlist.append(modDict)
	return dictlist

def tup2dict(mod):
	if mod:
		modDict = {'modID': mod[0],
				   'macAddr': mod[1],
				   'ipAddr': mod[2],
				   'posX': mod[3],
				   'posY': mod[4],
				   'nFans': mod[5],
			       'nLayers': mod[6],
			       'pwm': mod[7],
				   'rpm': mod[8],
				   'isConnected': mod[9],
				   'isFlashing': mod[10],
			       'isPowered': mod[11],
			       'lifePoints': mod[12],
			       'isRebooting': mod[13],
			       'isSendingRPM': mod[14]}
		return modDict
	return None

def decayModulesLife(con, cur):
	""" For all modules that are connected, decay their life points by 1 and set isConnected to 0
	if lifePoints <= 0"""
	with con:

		# Get modules where isConnected = 1
		conMods = getModulesConnected(con, cur)

		# For each connected modules get MAC address and lifePoints
		for mod in conMods:
			newLifePoints = mod['lifePoints']-1
			macAddr = mod['macAddr']

			#print macAddr, newLifePoints

			# Set new lifePoints value
			setCustomField(con, cur, 'lifePoints', newLifePoints, 'macAddr', macAddr)

			# If necessary set isConnected to 0 and restet state variables
			if newLifePoints < 1:
				s = '0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'
				setCustomField(con, cur, 'isConnected', 0, 'macAddr', macAddr)
				setCustomField(con, cur, 'pwm', s, 'macAddr', macAddr)
				setCustomField(con, cur, 'rpm', s, 'macAddr', macAddr)
				setCustomField(con, cur, 'isPowered', 0, 'macAddr', macAddr)
				#setCustomField(con, cur, 'isFlashing', 0, 'macAddr', macAddr)





