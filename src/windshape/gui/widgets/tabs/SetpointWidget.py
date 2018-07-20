from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

# Personal libraries
from ..inputs.EditInput import EditInput


class SetpointWidget(QWidget):
	"""Widget composed of four EditInput widgets.
	
	Get drone setpoint (x, y, z, yaw) from the user.
	
	Inherits from QWidget.
	
	Overrides: __init__
	"""
	X_MIN = -2000
	X_MAX = 2000
	
	Y_MIN = -2000
	Y_MAX = 2000
	
	Z_MIN = 0
	Z_MAX = 2000
	
	def __init__(self, parent):
		"""Creates inputs.
		
		Args:
			parent (QWidget): The parent widget to insert instance
		"""
		super(SetpointWidget, self).__init__(parent)
		
		# GridLayout for main layout
		self.layout = QGridLayout()
		self.setLayout(self.layout)
		
		# X input
		val = QDoubleValidator(self.X_MIN, self.X_MAX, 3)
		label = QLabel('X (mm):')
		self.x = EditInput(self, val)
		self.layout.addWidget(label, 0, 0)
		self.layout.addWidget(self.x, 0, 1)
		
		# Y input
		val = QDoubleValidator(self.Y_MIN, self.Y_MAX, 3)
		label = QLabel('Y (mm):')
		self.y = EditInput(self, val)
		self.layout.addWidget(label, 1, 0)
		self.layout.addWidget(self.y, 1, 1)
		
		# Z input
		val = QDoubleValidator(self.Z_MIN, self.Z_MAX, 3)
		label = QLabel('Z (mm):')
		self.z = EditInput(self, val)
		self.layout.addWidget(label, 2, 0)
		self.layout.addWidget(self.z, 2, 1)
		
		# Yaw input
		val = QDoubleValidator(-180, 180, 3)
		label = QLabel('Yaw (deg):')
		self.yaw = EditInput(self, val)
		self.layout.addWidget(label, 3, 0)
		self.layout.addWidget(self.yaw, 3, 1)
		
		# Stretch
		self.layout.setRowStretch(self.layout.rowCount(), 1)
		self.layout.setColumnStretch(self.layout.columnCount(), 1)
		
	def getValues(self):
		"""Returns the current setpoint (x, y, z, yaw) in mm, deg."""
		x = float(self.x.getValue())
		y = float(self.y.getValue())
		z = float(self.z.getValue())
		yaw = float(self.yaw.getValue())
		
		return x, y, z, yaw
		
	def isValid(self):
		"""Returns False if one of the fields is not valid."""
		
		if not self.x.isValid():
			return False
			
		if not self.y.isValid():
			return False
			
		if not self.z.isValid():
			return False
			
		if not self.yaw.isValid():
			return False
			
		return True
		
	def setValues(self, x, y, z, yaw):
		"""Set new setpoint.
		
		Args:
			x, y, z, yaw (float): Setpoint in mm, deg
		"""
		self.x.setValue(str(x))
		self.y.setValue(str(y))
		self.z.setValue(str(z))
		self.yaw.setValue(str(yaw))
