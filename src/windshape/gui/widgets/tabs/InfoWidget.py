from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

# Personal libraries
from ..inputs.ComboInput import ComboInput


class InfoWidget(QWidget):
	"""Widget composed of a ComboInput and a QTextEdit widgets.
	
	Display info chosen with combo in text edit.
	
	Inherits from QWidget.
	
	Overrides: __init__
	"""
	
	def __init__(self, parent, sources):
		"""Creates widgets.
		
		Args:
			parent (QWidget): The parent widget to insert instance
			sources (str list): List of the sources of display
		"""
		super(InfoWidget, self).__init__(parent)
		
		# Vertical layout
		self.layout = QVBoxLayout()
		self.setLayout(self.layout)
		
		# Create combo with label and info sources
		hbox = QHBoxLayout()
		label = QLabel('Source:')
		self.combo = ComboInput(self, sources)
		hbox.addWidget(label)
		hbox.addWidget(self.combo)
		hbox.addStretch(1)
		self.layout.addLayout(hbox)
		
		# Create text edit with read only attribute
		self.text = QTextEdit()
		self.text.setReadOnly(True)
		self.layout.addWidget(self.text)
		
	def getSource(self):
		"""Returns the current source selected by user (str)."""
		return str(self.combo.getValue())
		
	def setText(self, text):
		"""Displays the given text in the text edit.
		
		Args:
			text (str): Text to display
		"""
		self.text.setText(str(text))
