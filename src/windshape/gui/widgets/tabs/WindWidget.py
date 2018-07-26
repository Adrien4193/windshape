from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

# ROS main library
import rospy

# Personal libraries
from ..inputs.EditInput import EditInput


class WindWidget(QWidget):
	"""Widget composed of one EditInput widget.
	
	Get PWM value of the fans array from the user.
	
	Inherits from QWidget.
	
	Overrides: __init__
	"""
	
	def __init__(self, parent):
		"""Creates input.
		
		Args:
			parent (QWidget): The parent widget to insert instance
		"""
		super(WindWidget, self).__init__(parent)
		
		# GridLayout for main layout
		self.layout = QGridLayout()
		self.setLayout(self.layout)
		
		# Creates PWM input
		val = None
		label = QLabel('Wind function (%):')
		self.pwm = EditInput(self, val, 100)
		self.layout.addWidget(label, 0, 0)
		self.layout.addWidget(self.pwm, 0, 1)
		
		# Creates checkbox
		self.auto = QCheckBox('Activate auto-wind')
		self.auto.setChecked(rospy.get_param('~fansarray/auto_wind'))
		self.layout.addWidget(self.auto, 1, 0)
		
		# Stretch
		self.layout.setRowStretch(self.layout.rowCount(), 1)
		#self.layout.setColumnStretch(self.layout.columnCount(), 1)
	
	def getAutoWind(self):
		"""Returns True if auto-wind is checked."""
		return self.auto.isChecked()
	
	def getValue(self):
		""" Return the PWM value in % (float) """
		return self.pwm.getValue()
		
	def isValid(self):
		""" Return True if the input is valid. """
		return self.pwm.isValid()
		
	def setValue(self, value):
		""" Set the PWM value to new one.
		
		Args:
			value (float): New PWM value from 0 to 100.
		"""
		self.pwm.setValue(value)
