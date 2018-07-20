import os
import time

# QT
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QIcon

# Custom combo
from ..widgets.inputs.ComboInput import ComboInput


class ComboDialog(QDialog):
	"""Dialog window to choose between a list of items.
	
	Inherits from QDialog.
	
	Overrides __init__, keyPressEvent, getResult
	"""
	
	def __init__(self, parent, label, items, default=None):
		"""Creates combo and buttons.
		
		Args:
			parent (QWidget): The parent widget to insert instance
			label (str): Name of the list of items
			items (str list): Items to choose between
			default (str): Item of the list to start with
		"""
		
		super(ComboDialog, self).__init__(parent)
		
		# Icon
		path = os.path.dirname(os.path.abspath(__file__))
		path = os.path.dirname(path)
		self.setWindowIcon(QIcon(path + '/icons/start.png'))
		
		# Main layout
		self.layout = QVBoxLayout()
		self.setLayout(self.layout)
		
		# Combo Layout
		hbox = QHBoxLayout()
		self.layout.addLayout(hbox)
		
		# Combo choice
		label = QLabel(label)
		self.combo = ComboInput(self, items)
		hbox.addWidget(label)
		hbox.addWidget(self.combo)
		
		# Default value
		if default is not None:
			self.combo.setValue(default)

		# Buttons layout
		hbox = QHBoxLayout()
		self.layout.addLayout(hbox)
		
		# OK
		okButton = QPushButton('OK')
		okButton.clicked.connect(self.accept)
		hbox.addWidget(okButton)
		
		# Cancel
		cancelButton = QPushButton('Cancel')
		cancelButton.clicked.connect(self.reject)
		hbox.addWidget(cancelButton)
			
	def getResult(self):
		"""Returns the selected item."""
		return self.combo.getValue()
		
	def keyPressEvent(self, event):
		"""Press Enter to validate and Escape to cancel."""
		
		# Enter
		if event.key() == Qt.Key_Return or event.key() == Qt.Key_Enter:
			self.accept()
		
		# Escape
		if even.key() == Qt.Key_Escape:
			self.reject()
