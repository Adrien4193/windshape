from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


class EditInput(QLineEdit):
	"""Custom QLineEdit.
	
	This widget has for purpose to get values from user.
	
	Inherits from QLineEdit.
	
	Overrides: __init__
	"""
	
	def __init__(self, parent, validator=None, max_len=20):
		"""Creates edit text and set 0 as value.
		
		Args:
			parent (QWidget): The parent widget to insert instance
			validator=None (QValidator): Validator used for inputs
			max_len=20 (int): Maximum number of digits
		"""
		super(EditInput, self).__init__(parent)
		
		# Initializes to 0 and validity
		self.setText('0')
		self.valid = True
		
		# Applies max_len parameter and validator with callback
		self.setMaxLength(max_len)
		
		if validator is not None:
			self.setValidator(validator)
			self.textChanged.connect(self.__checkValue)
		
	def getValue(self):
		"""Returns the current value of the input (str)."""
		return str(self.text())
		
	def isValid(self):
		"""Returns True if the text is validated by validator."""
		return self.valid
		
	def setValue(self, value):
		"""Set a new value to the edit text.
		
		Args:
			value (str): New value for the spin box
		"""
		self.setText(str(value))
		
	def __checkValue(self, *args, **kwargs):
		"""Called when text is modified to check validity."""
		# Validate input
		state = self.validator().validate(self.text(), 0)[0]
		
		# Set valid and white background if correct
		if state == QValidator.Acceptable:
			self.valid = True
			color = '#ffffff' # white
		
		# Else reset valid and set red background
		else:
			self.valid = False
			color = '#f6989d' # red
		
		# Change background
		self.setStyleSheet('QLineEdit { background-color: %s }' % color)
