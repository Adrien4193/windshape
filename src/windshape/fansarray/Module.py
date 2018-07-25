class Module(dict):
	"""Class for modules representation.
	
	Keys are modules attributes.
	
	Inherits from: dict.
	
	Overrides: __init__, __del__, __str__.
	"""
	
	def __init__(self, module):
		"""Initializes keys and values.
		
		Args:
			module (tuple): Module attributes
			
		Structure:
		
			modID: 0
			macAddr: 1
			ipAddr: 2
			posX: 3
			posY: 4
			nFans: 5
			nLayers: 6
			pwm: 7
			rpm: 8
			isConnected: 9
			isFlashing: 10
			isPowered: 11
			lifePoints: 12
			isRebooting: 13
			isSendingRPM: 14
		"""
		super(Module, self).__init__()
		
		# Attributes
		self['modID'] = module[0]
		
		self['macAddr'] = module[1]
		self['ipAddr'] = module[2]
		
		self['posX'] = module[3]
		self['posY'] = module[4]
		
		self['nFans'] = module[5]
		self['nLayers'] = module[6]
		
		self['rpm'] = module[7]
		self['pwm'] = module[8]
		
		self['isConnected'] = module[9]
		self['isFlashing'] = module[10]
		self['isPowered'] = module[11]
		self['lifePoints'] = module[12]
		
		self['isRebooting'] = module[13]
		self['isSendingRPM'] = module[14]
		
		self.__update = {}
		for key in self.keys():
			self.__update[key] = False
		
	def __del__(self):
		"""Does nothing special."""
		pass
		
	def __str__(self):
		"""Display the modules attributes."""
		string = ['Module '+str(self['modID'])+':']
		
		for attribute, value in self.items():
			string.append('{}: {}'.format(attribute, value))
			
		return '\n    '.join(string)
	
	#
	# Public methods to update attributes.
	#
	
	def needsUpdate(self, attribute):
		"""Returns True if the attribute (str) needs to be updated."""
		if attribute not in self.keys():
			rospy.logerr('Wrong attribute: %s', attribute)
			
		elif self.__update[attribute]:
			self.__update[attribute] = False
			return True
		
		return False
		
	def setAttribute(self, attribute, value):
		"""Writes module in database."""
		if attribute not in self.keys():
			rospy.logerr('Wrong attribute: %s', attribute)
			return
		
		self[attribute] = value
		self.__update[attribute] = True
