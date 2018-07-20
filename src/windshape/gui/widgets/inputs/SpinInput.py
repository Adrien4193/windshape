from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


class SpinInput(QSpinBox):
	"""Widget composed of a QLabel and a QSpinBox (int).
	
	This widget has for purpose to get numeric values from user.
	
	Inherits from QSpinBox.
	
	Overrides: __init__
	"""
	
	def __init__(self, parent, minVal, maxVal):
		"""Creates label and spin with validation.
		
		Args:
			parent (QWidget): The parent widget to insert instance
			minVal (int): Minimum value of the input
			maxVal (int): Maximum value of the input
		"""
		
		super(SpinInput, self).__init__(parent)
		
		self.setRange(minVal, maxVal)
		
	def getValue(self):
		"""Returns the current value of the spin box (int)."""
		return self.value()
		
	def setValue(self, value):
		"""Set a new value to the spin box.
		
		Args:
			value (int): New value for the spin box
		"""
		super(SpinInput, self).setValue(value)
