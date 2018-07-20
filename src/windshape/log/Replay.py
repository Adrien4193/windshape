import os
import sys

# Dialog with combo
from ..gui.dialogs.ComboDialog import ComboDialog
# Dialog with plot
from ..gui.dialogs.ReplayDialog import ReplayDialog


class Replay(ReplayDialog):
	"""Plots the content of a file chosen with dialog.
	
	Runs as stand-alone in a QApplication.
	
	Inherits from ReplayDialog.
	
	Overrides: __init__.
	"""
	
	def __init__(self, parent=None):
		"""Make user choose a measurement file and plot it."""

		# Log directory
		directory = os.path.dirname(os.path.abspath(__file__))+'/files'

		# Selects command and pose files
		files = []
		if os.path.isdir(directory):
			for fileName in os.listdir(directory):
				if 'command' in fileName or 'pose' in fileName:
					files.append(fileName)
			files.sort()
		
		# Creates dialog to choose file from list
		dialog = ComboDialog(parent, 'Data file', files)
		
		# Loads file if OK pressed
		if dialog.exec_():
			fileName = dialog.getResult()
			super(Replay, self).__init__(parent, directory+'/'+fileName)
		
		# Else kills QApplication if no more widget running
		elif parent == None:
			sys.exit()

