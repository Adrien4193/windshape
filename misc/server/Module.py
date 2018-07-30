import time
import threading

import dbapi


class Module(dict):
	
	rate = 50.0 # [Hz]
	verbose = False
	
	def __init__(self, con, cur, modID, macAddr, x, y):
		
		# Initialize parent
		super(Module, self).__init__()
		
		# Connection to DB
		self.con = con
		self.cur = cur
		
		# Attributes
		self['modID'] = modID
		
		self['macAddr'] = macAddr
		self['ipAddr'] = '127.0.0.1'
		
		self['posX'] = x
		self['posY'] = y
		
		self['nFans'] = 9
		self['nLayers'] = 2
		
		self['rpm'] = ','.join(18*['0'])
		self['pwm'] = ','.join(18*['0'])
		
		self['isConnected'] = '0'
		self['isFlashing'] = '0'
		self['isPowered'] = '0'
		self['lifePoints'] = 0
		
		self['isRebooting'] = '0'
		self['isSendingRPm'] = '0'
		
	def __del__(self):
		pass
		
	def __str__(self):
		header = 'Module '+str(self['modID'])+':\n'
		
		attributes = []
		
		for attribute in self.keys():
			attributes.append(str(attribute)+': '+str(self[attribute]))
			
		stats = '\n'.join(attributes)
		
		return header + stats
		
	def decayLifePoints(self):
		self['lifePoints'] -= 1
		
		if self['lifePoints'] <= 0:
			self['lifePoints'] = 0
			self.setAttribute('isConnected', '0')
			
	def resetLifePoints(self):
		self['lifePoints'] = 100
		self.setAttribute('isConnected', '1')
		
	def setAttribute(self, attribute, value):
		
		if self[attribute] != value:
			self[attribute] = value
			self.writeAttribute(attribute)
			
	def writeAttribute(self, attribute):

		with self.con:
			cmd = 'UPDATE modules SET '+attribute+'=%s WHERE modID=%s'
			values = [self[attribute], self['modID']]
			self.cur.execute(cmd, values)
				
	def loadAttribute(self, attribute):
		
		with self.con:
			cmd = 'SELECT '+attribute+' FROM modules WHERE modID=%s'
			self.cur.execute(cmd, [self['modID']])
			value = self.cur.fetchone()[0]
			self[attribute] = value
			
		return value
			
	def insertToDB(self):
		
		with self.con:
			
			cmd = """
			
			INSERT INTO modules 
				(modID, macAddr, ipAddr, posX, posY, nFans, nLayers, 
					pwm, rpm, isConnected, isFlashing, isPowered,
					lifePoints, isRebooting, isSendingRPM)
			
			VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s);""", 
			
			values = (self['modID'], self['macAddr'], self['ipAddr'],
				self['posX'], self['posY'], self['nFans'], self['nLayers'], 
				self['pwm'], self['rpm'], 0, 0, 0, 100, 0, 0)
			
			self.cur.execute(cmd, values)
