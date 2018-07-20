import os
import time

# QT
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

# Plot data
from ..widgets.plots.PlotWidget import PlotWidget


class ReplayDialog(QDialog):
	"""Plots data recorded in a file in a QDialog window.
	
	Inherits from QDialog.
	
	Overrides: __init__
	"""
	
	def __init__(self, parent, fileName):
		"""Extracts data from file and plot it.
		
		Args:
			parent (QWidget): The parent widget.
			fileName (str): Path of the file with data to plot.
		"""
		super(ReplayDialog, self).__init__(parent)
		
		# Title and geometry
		self.setWindowTitle('Replay of '+str(fileName))
		self.setGeometry(600, 100, 1000, 600)
		
		# Icon
		path = os.path.dirname(os.path.abspath(__file__))
		path = os.path.dirname(path)
		self.setWindowIcon(QIcon(path + '/icons/start.png'))
		
		# Main layout (vertical)
		self.layout = QVBoxLayout()
		self.setLayout(self.layout)
		
		# Widget to plot the data
		self.plot = PlotWidget(self)
		self.layout.addWidget(self.plot)
		
		# Extract the data from file
		self.load(fileName)
		
		# Draw plot and show window
		self.plot.draw()
		self.show()
		
	def load(self, fileName):
		"""Opens file and extract data."""

		try:
			file_ = open(fileName, 'r')
		except IOError as e:
			print e
			return
		
		# Extracts and plot data
		try:
			self.extractData(file_)
		except:
			print 'Error while extracting data in ', fileName
		
	def extractData(self, file_):
		"""Extracts data from file and add to plot.
		
		FORMAT:
		
		time: <time_stamp>
		    <source>:
		        <variable>: <value> <unit>
		"""
		# Initializes data
		timeStamp = source = None
		
		# Extract data if set
		extract = False
		
		# Read file
		for line in file_:
			
			# Remove '\n'
			line = line.replace('\n', '')
			
			# Empty line = end of datapoint description
			if len(line) < 2:
				continue
				
			# Time stamp -> New datapoint
			if line.startswith('time: '):
				timeStamp = float(line.split(': ', 2)[1])
				continue
				
			if timeStamp is None:
				continue
			
			if line.startswith(4*' ') and ':' in line:
				
				# Source
				if not line.startswith(8*' '):
					source = line[4:line.index(':')]
					continue
				
				# Variable
				elif source is not None and ': ' in line:
					data = line.replace(8*' ', '').split(': ', 2)
					variable = data[0].title()
					value, unit = data[1].split(' ', 2)
			else:
				continue
			
			# Add variable if needed to display units
			if variable not in self.plot.datapoints.keys():
				self.plot.addVariable(variable, unit)
			
			# Plot datapoint
			self.plot.appendData(variable, source,
									timeStamp, float(value))
