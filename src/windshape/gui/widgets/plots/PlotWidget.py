import time

# Graphical library
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

# Plot library
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar


class PlotWidget(QWidget):
	"""Widget used to plot data.
	
	Inherits from QWidget.
	
	Overrides __init__.
	"""
	
	GRAPH_SPAN_Y = 2000
	GRAPH_SPAN_X = 20
	
	# INITIALIZATION
	####################################################################
	
	def __init__(self, parent):
		"""Creates plot figure and data points lists.
		
		Args:
			parent (QWidget): The parent widget to insert instance
		"""
		super(PlotWidget, self).__init__(parent)
		
		# Structure: dict -> dict -> list
		#
		# Access: datapoints[variable][source][number]
		#
		# Example: datapoints['X']['Mocap'][12] = value # (float)
		
		# X and Y data points list (see explanation above)
		self.datapoints = {}
		
		# Units stored by variable
		self.units = {}
		
		# Current variable to plot
		self.variable = None
		
		# Create figure
		self.initUI()

	def initUI(self):
		"""Creates plot figure."""
		
		# Vertical layout
		self.layout = QVBoxLayout()
		self.setLayout(self.layout)
		
		# New figure
		self.figure = plt.figure()
		
		# New plot (axes)
		self.axes = self.figure.add_subplot(111)
		
		# Widget for drawing figure
		self.canvas = FigureCanvas(self.figure)
		
		# Header (variable choice + toolbar)
		self.createHeader()
		
		# Add canvas after toolbar to Vlayout
		self.layout.addWidget(self.canvas)
		
	def createHeader(self):
		"""Creates header (QComboBox + Toolbar)."""
		
		# Header layout
		hbox = QHBoxLayout()
		
		# Create combo with callback
		self.combo = QComboBox(self)
		self.combo.activated[str].connect(self.onActivated)
		hbox.addWidget(QLabel('Variable:'))
		hbox.addWidget(self.combo)
		
		# Create toolbar and bind with canvas
		toolbar = NavigationToolbar(self.canvas, self)
		hbox.addWidget(toolbar)
		
		# Add header to main layout
		self.layout.addLayout(hbox)
		
	def onActivated(self, item):
		"""Combo callback to change displayed variable."""
		self.variable = str(item)
		self.draw()
		
	# DATA PLOTTING
	####################################################################
		
	def addSource(self, name):
		"""Creates new source for a variable.
		
		Args:
			name (str): Name of the source (ex: 'Mocap') 
		"""
		for variable in self.datapoints.keys():
			
			if name not in self.datapoints[variable].keys():
				self.datapoints[variable][name] = []
			
	def addVariable(self, name, unit='-'):
		"""Creates new variable for plotting.
		
		Args:
			name (str): Name of the variable
			unit (str): Unit of the variable (for axis labelling)
		"""
		# Create sub-dictionary corresponding to new variable
		self.datapoints[name] = {}
		
		# Add unit to dictionary
		self.units[name] = unit
		
		# Add variable to combo choice
		self.combo.addItem(name)
		
		# Set as active variable if none
		if self.variable == None:
			self.variable = name
	
	def appendData(self, variable, source, x, y):
		"""Adds a data point to the graph.
		
		Args:
			variable (str): Name of the variable of the point (ex: 'X')
			source (str): Name of the source of the point (ex: 'Mocap')
			x (float): X value of the data point
			y (float): Y value of the data point
		"""
		# Add variable to x and y keys if not done
		#(better to declare manually to have units)
		if not variable in self.datapoints.keys():
			self.addVariable(variable)
		
		# Add source to all x and y variables if not done
		if not source in self.datapoints[variable].keys():
			self.addSource(source)
		
		# Append datapoint as tuple
		self.datapoints[variable][source].append((x, y))

	# DRAWING
	####################################################################
		
	def draw(self):
		"""Redraws the whole figure."""
		
		# Clear the figure from all data
		self.figure.clear()
		
		# Re-create plot
		self.axes = self.figure.add_subplot(111)
		
		# Plot data with source as label
		if self.variable in self.datapoints.keys():
		
			for source in self.datapoints[self.variable].keys():
				line = self.datapoints[self.variable][source]
				
				xs = [datapoint[0] for datapoint in line]
				ys = [datapoint[1] for datapoint in line]

				self.axes.plot(xs, ys, label=source)
		
		# Setup title, grid, labels
		self.setupLayout()
		
		# Rescale plot from XY bounds
		self.resetViewport()
		
		# Redraw plot
		self.canvas.draw()
		
	def getBounds(self):
		"""Returns max X and Y in all datapoints."""
		
		# Initialize max and min for X and Y
		max_x = PlotWidget.GRAPH_SPAN_X
		max_y = 1
		
		# Safety if no variable already set
		if not self.variable in self.datapoints.keys():
			return max_x, max_y
		
		# Update Y bounds as a function of current variable unit
		unit = self.units[self.variable]
		
		if unit == 'mm':
			max_y = PlotWidget.GRAPH_SPAN_Y
		elif unit == 'deg':
			max_y = 180
		elif unit == '%':
			max_y = 100
		
		# Get max and min in all sources
		for source in self.datapoints[self.variable].keys():
			line = self.datapoints[self.variable][source]
			
			xs = [datapoint[0] for datapoint in line]
			ys = [datapoint[1] for datapoint in line]
			
			if len(xs) > 0 and max(xs) > max_x:
				max_x = max(xs)
				
			if len(ys) > 0 and max(ys) > max_y:
				max_y = max(ys)
		
		return max_x, max_y
		
	def resetViewport(self):
		"""Resets the viewport to XY bounds."""
		max_x, max_y = self.getBounds()
		
		self.axes.set_xbound(0, max_x) # Start always from 0
		self.axes.set_ybound(-max_y, max_y) # Symetric view
		
	def setupLayout(self):
		"""Setup title, grid, axis labels and legend."""
		
		# Title
		self.axes.set_title('Drone position '+str(self.variable))
		
		# Grid
		self.axes.grid(True)
		
		# Units
		if self.variable in self.units.keys():
			unit = self.units[self.variable]
		else:
			unit = '-'
		
		# XY labels
		self.axes.set_xlabel('Time (s)')
		self.axes.set_ylabel('Measurement ('+str(unit)+')')
		
		# Legend
		plt.legend(loc=2, prop={'size': 10})
