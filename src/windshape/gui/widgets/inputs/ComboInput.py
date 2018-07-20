from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


class ComboInput(QComboBox):
	"""Custom QComboBox.
	
	This widget has for purpose to get an item among a list from user.
	
	Inherits from QComboBox.
	
	Overrides: __init__
	"""
	
	def __init__(self, parent, items):
		"""Creates combo from parameters.
		
		Args:
			parent (QWidget): The parent widget to insert instance
			items (str list): Items to fill the combo
		"""
		
		super(ComboInput, self).__init__(parent)
		
		# Memorize items list
		self.items = items
		
		# Set current value to first item or empty string if no item
		self.value = str(items[0]) if len(items) > 0 else ''
		
		# Fill combo with new items
		for item in items:
			self.addItem(str(item))
		
		# Callback for activation
		self.activated[str].connect(self.__onActivated)
			
	def getItems(self):
		"""Returns the current list of items in combo (str list). """
		return self.items
		
	def getValue(self):
		"""Returns current value (str). """
		return self.value
		
	def setItems(self, items):
		"""Changes the current combo items list.
		
		Args:
			items (str list): New items list for combo
		"""
		self.items = items
		
		# Remove old items
		self.clear()
		
		# Fill combo with new items
		for item in items:
			self.addItem(str(item))
		
		# Keep old value and set as active item
		self.setValue(self.value)
		
	def setValue(self, value):
		"""Set current value to parameter.
		
		Args:
			value (str): New value of widget.
		"""
		self.value = str(value)
		
		# Set current index of combo to new value if in list
		if value in self.items:
			index = self.items.index(value)
			
		# Else add to list and set as active item
		else:
			self.addItem(str(value))
			index = self.count() - 1
		
		self.setCurrentIndex(index)
		
	def __onActivated(self, item):
		"""Called automatically each time the combo is modified."""
		self.value = str(item)
