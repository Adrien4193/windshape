import os
import sys
import datetime

# ROS main library
import rospy


class Recorder(object):
	"""Records data in log files.
	
	Inherits from: object.
	
	Overrides: __init__, __del__.
	"""
	
	# INITIALIZER AND DESTRUCTOR
	####################################################################
	
	def __init__(self):
		"""Opens log files."""
		
		# Log files base names
		fileNames = ['status', 'pose', 'command']
		
		# Files and time reference for stamping
		self.__files = {}
		self.__refs = {}
		
		# Creates files
		for name in fileNames:
			self.__files[name] = self.__openFile(name)
			self.__refs[name] = None
		
	def __del__(self):
		"""Closes log files properly."""
		for file_ in self.__files.values():
			if file_ is not None and not file_.closed:
				file_.close()
	
	# RECORD DATA
	####################################################################
		
	def record(self, fileName, text):
		"""Writes text in log file with time stamp.
		
		Args:
			fileName (str): Name of the log file-
			text (str): Text to log.
		"""
		if self.__refs[fileName] is None:
			self.__refs[fileName] = rospy.get_time()
			timeStamp = 0
		else:
			timeStamp = rospy.get_time() - self.__refs[fileName]
		
		text = text.replace('\n', '\n    ')
		text = 'time: '+str(timeStamp)+'\n    '+text+'\n\n'
		self.__files[fileName].write(text)
	
	# FILES MANAGEMENT
	####################################################################
	
	def __openFile(self, name):
		"""Opens file from name in ./files.
		
		Args:
			name (str): File name without path, index or extension
		"""
		directory = os.path.dirname(os.path.abspath(__file__))+'/files'
		
		# Creates folder if not done
		if not os.path.isdir(directory):
			os.makedirs(directory)
			
		# Gets index to append to file name
		index = self.__getFileIndex(directory, name)
		
		# Creates file
		try:
			file_ = open(directory+'/'+name+str(index)+'.txt', 'w')
		except IOError as error:
			rospy.logerr(error)
			return None
		
		# Dates the file
		now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')
		date = 'Date: ' + now + '\n\n\n'
		file_.write(date)
		
		return file_
		
	def __getFileIndex(self, directory, name):
		"""Returns the index of files with same name in directory.
		
		Args:
			directory (str): The directory full path to search files.
			name (str): The base name of the files to consider.
		"""
		files = os.listdir(directory)
		
		# Gets indexes of the files with same name
		indexes = []
		for fileName in files:
			
			# Select only files with same base name
			if name not in fileName:
				continue
			
			# nameXX -> XX
			index = fileName.replace(name, '').replace('.txt', '')
			
			# Add to list if digit
			if index.isdigit():
				indexes.append(int(index))
		indexes.sort()
		
		# Removes older files if max_files is reached
		maxFiles = rospy.get_param('~max_files')
		while len(indexes) >= maxFiles:
			os.remove(directory+'/'+name+str(indexes[0])+'.txt')
			del indexes[0]
		
		# Gets max index
		if len(indexes) > 0:
			num = indexes[-1]
		else:
			num = -1
		
		# Returns next index
		return num + 1
