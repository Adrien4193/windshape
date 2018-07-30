import re


# Functions of the configurator allows to modelize the physical equipment numerically

def readConfigFile(filePath):
	""" Read the config file and generate a dictionnary containing an entry for 
	every modules of the installation. """

	modules_attributes_list = []
	confFile =  open(filePath, "r")
	for i, line in enumerate(confFile.readlines()):

		# Remove everything that is written after "#" character (comments)
		line = line.split("#")[0]
		line = line.split("//")[0]
		line = line.split("$")[0]

		# Remove special characters
		line = re.sub('[!@#$\0\\n ]','',line)

		# Get the MAC addresses and the modules number
		words = line.split(",")

		if len(words) == 4:
			modID = int(words[0])
			posY = int(words[1])
			posX = int(words[2])
			macAddr = words[3]
			modules_attributes_list.append((modID, posY, posX, macAddr))

		elif len(words) < 2:
			pass

		else :
			raise AttributeError("Wrong formatting of the MAC file.")

	return modules_attributes_list

def getModulesStructure(configFile="modules.conf", fanNumbers=9, fanLayers=2):
	""" Create a dictionnary structure of the physical modules and their arrangement 
	given a configuration file and modules design parameters. """

	# Read configuration file and generate a list of modules attributes
	modAttr_list = readConfigFile(configFile)

	# Generate a module dictionnary structure
	modules_dict = {}
	for modAttr in modAttr_list:
		modID, modPosY, modPosX, macAddr = modAttr
		ipAddr = ""
		fanNumbers = 9
		fanLayers = 2

		z = ''
		for ifan in range(fanNumbers):
			for ilay in range(fanLayers):
				z += "0,"
		z = z[:-1]

		modules_dict[modID] = {'posX': modPosX, 
							   'posY': modPosY, 
							   'macAddr': macAddr, 
							   'ipAddr':ipAddr, 
							   'nFans': fanNumbers, 
							   'nLayers': fanLayers,
							   'pwm': z,
							   'rpm': z}
	return modules_dict
