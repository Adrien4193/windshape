from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

# Personal libraries
from ..inputs.SpinInput import SpinInput


class AttitudeWidget(QWidget):
	"""Widget composed of four SpinInput widgets.
	
	Allow the user to send manually attitude to the drone.
	
	Inherits from QWidget.
	
	Overrides: __init__
	"""
	
	def __init__(self, parent):
		"""Creates inputs.
		
		Args:
			parent (QWidget): The parent widget to insert instance
		"""
		
		super(AttitudeWidget, self).__init__(parent)
		
		# GridLayout for main layout
		self.layout = QGridLayout()
		self.setLayout(self.layout)
		
		# Roll (deg)
		label = QLabel('Roll (deg):')
		self.roll = SpinInput(self, -180, 180)
		self.roll.setValue(0)
		self.layout.addWidget(label, 0, 0)
		self.layout.addWidget(self.roll, 0, 1)
		
		# Pitch (deg)
		label = QLabel('Pitch (deg):')
		self.pitch = SpinInput(self, -180, 180)
		self.pitch.setValue(0)
		self.layout.addWidget(label, 1, 0)
		self.layout.addWidget(self.pitch, 1, 1)
		
		# Yaw (deg)
		label = QLabel('Yaw (deg):')
		self.yaw = SpinInput(self, -180, 180)
		self.yaw.setValue(0)
		self.layout.addWidget(label, 2, 0)
		self.layout.addWidget(self.yaw, 2, 1)
		
		# Thrust (%)
		label = QLabel('Thrust (%):')
		self.thrust = SpinInput(self, 0, 100)
		self.thrust.setValue(0)
		self.layout.addWidget(label, 3, 0)
		self.layout.addWidget(self.thrust, 3, 1)
		
		# Stretch
		self.layout.setRowStretch(self.layout.rowCount(), 1)
		self.layout.setColumnStretch(self.layout.columnCount(), 1)
		
	def getValues(self):
		"""Returns the current values of attitude (int tuple)."""
		roll = self.roll.getValue()
		pitch = self.pitch.getValue()
		yaw = self.yaw.getValue()
		thrust = self.thrust.getValue()
		
		return roll, pitch, yaw, thrust
		
	def setValues(self, roll, pitch, yaw, thrust):
		"""Changes the values of the manual inputs.
		
		Args:
			roll (int): Roll trim (deg)
			pitch (int): Pitch trim (deg)
			yaw (int): Yaw trim (deg)
			thrust (float): Thrust trim (%)
		"""
		self.roll.setValue(int(roll))
		self.pitch.setValue(int(pitch))
		self.yaw.setValue(int(yaw))
		self.thrust.setValue(int(thrust))
