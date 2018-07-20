import math

# Graphical libraries
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

# Setpoint choice
from tabs.SetpointWidget import SetpointWidget
# PWM choice
from tabs.WindWidget import WindWidget
# Drone info display
from tabs.InfoWidget import InfoWidget
# Real-time plot
from plots.RealTimePlotWidget import RealTimePlotWidget
# Drone settings
from tabs.SettingsWidget import SettingsWidget
# Drone manual attitude control
from tabs.AttitudeWidget import AttitudeWidget


class ControlPanel(QTabWidget):        
	"""Main widget of the interface.
	
	Get drone setpoint, fans array PWM value, displays info about the 
	drone state, plots drone pose in real time, gets drone parameters,
	send attitude to the drone.
	
	Tabs:
		- Setpoint
		- Attitude
		- Wind
		- Info
		- Plot
		- Settings
	
	Inherits from QTabWidget.
	
	Overrides: __init__.
	"""
	
	# INITIALIZATION
	####################################################################

	def __init__(self, parent):
		"""Creates all tabs and initialize UI.
		
		Args:
			parent (QWidget): The parent widget to insert instance
		"""
		super(ControlPanel, self).__init__(parent)
		
		# Tab 0
		self.createSetpointTab()
		# Tab 1
		self.createAttitudeTab()
		# Tab 2
		self.createWindTab()
		# Tab 3
		self.createInfoTab()
		# Tab 4
		self.createPlotTab()
		# Tab 5
		self.createSettingsTab()
		
	# TABS CREATION
	####################################################################
		
	def createSetpointTab(self):
		"""Creates tab to choose drone setpoint (SetpointWidget)."""
		self.setpoint = SetpointWidget(self)
		self.addTab(self.setpoint, 'Setpoint')
		
	def createAttitudeTab(self):
		"""Creates tab to trim the drone (TuningWidget)."""
		self.attitude = AttitudeWidget(self)
		self.addTab(self.attitude, 'Attitude')
	
	def createWindTab(self):
		"""Creates tab to set PWM to fans array (WindWidget)."""
		self.wind = WindWidget(self)
		self.addTab(self.wind, 'Wind')

	def createInfoTab(self):
		"""Creates tab to display general information (InfoWidget)."""
		self.info = InfoWidget(self, ['Info', 'Pose', 'Command'])
		self.addTab(self.info, 'Info')
	
	def createPlotTab(self):
		"""Creates tab to plot drone pose (RealTimePlotWidget)."""
		self.plot = RealTimePlotWidget(self)
		self.addTab(self.plot, 'Plot')
		
		# Initializes variables to have units
		self.plot.addVariable('X', 'mm')
		self.plot.addVariable('Y', 'mm')
		self.plot.addVariable('Z', 'mm')
		self.plot.addVariable('Roll', 'deg')
		self.plot.addVariable('Pitch', 'deg')
		self.plot.addVariable('Yaw', 'deg')
	
	def createSettingsTab(self):
		"""Create tab to select drone parameters (SettingsWidget)."""
		self.settings = SettingsWidget(self)
		self.addTab(self.settings, 'Settings')
		
	# ATTRIBUTES GETTERS
	####################################################################
	
	def getSetpoint(self):
		"""Returns the setpoint selected in setpoint tab."""
		x, y, z, yaw = self.setpoint.getValues()
		return x/1000, y/1000, z/1000, math.radians(yaw)
		
	def getAttitude(self):
		"""Returns attitude set manually."""
		r, p, y, t = self.attitude.getValues()
		return math.radians(r), math.radians(p), math.radians(y), t/100.0
		
	def getPWM(self):
		"""Returns the PWM value selected in wind tab."""
		return self.wind.getValue()
		
	def getAutoWind(self):
		"""Returns True if the auto wind should be activated."""
		return self.wind.getAutoWind()
		
	def getSource(self):
		"""Returns the source of information asked in info tab."""
		return self.info.getSource()
		
	def getConnectionParameters(self):
		"""Returns FCU IP, protocol and stack from settings tab."""
		return self.settings.getConnectionParameters()
		
	def getMocapParameters(self):
		"""Returns server IP, drone and target ID from settings tab."""
		return self.settings.getMocapParameters()
		
	def getControlParameters(self):
		"""Returns the task and the control mode from settings tab."""
		return self.settings.getControlParameters()
		
	# VALIDITY CHECKS
	####################################################################
	
	def setpointValid(self):
		"""Returns True if the setpoint selected is valid."""
		return self.setpoint.isValid()
		
	def pwmValid(self):
		"""Returns True if the PWM value selected is valid."""
		return self.wind.isValid()
		
	# UPDATE
	####################################################################
	
	def displayInfo(self, text):
		"""Set text (str) into info tab."""
		self.info.setText(text)
	
	def plotData(self, source, data):
		"""Plots the data (list) with source (str) label."""
		self.plot.appendData('X', source, 1000*data[0])
		self.plot.appendData('Y', source, 1000*data[1])
		self.plot.appendData('Z', source, 1000*data[2])
		self.plot.appendData('Roll', source, math.degrees(data[3]))
		self.plot.appendData('Pitch', source, math.degrees(data[4]))
		self.plot.appendData('Yaw', source, math.degrees(data[5]))
	
	def updateBodies(self, bodies):
		"""Updates the list of rigid bodies in settings tab."""
		if self.settings.getBodies() != bodies:
			self.settings.setBodies(bodies)
