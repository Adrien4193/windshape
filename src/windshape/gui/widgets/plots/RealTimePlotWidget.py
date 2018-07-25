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

# Personal library
from PlotWidget import PlotWidget

	
class RealTimePlotWidget(PlotWidget):
	"""Plots data in real time.
	
	Inherits from PlotWidget.
	
	Overrides: __init__, addSource, appendData, draw
	"""
	
	## Animation rate
	FRAMERATE = 30.0
	
	def __init__(self, parent):
		"""Creates parent and start animation.
		
		Args:
			parent (QWidget): The parent widget to insert instance
		"""
		super(RealTimePlotWidget, self).__init__(parent)
		
		# Auto X axis
		self.t_start = None
		
		# One line per source (for current variable)
		self.lines = {}
		
		# Animation to optimize real time display
		self.anim = animation.FuncAnimation(self.figure, self.refresh,  
							interval=1000/RealTimePlotWidget.FRAMERATE)
		
	def addSource(self, source):
		"""Adds new source to display (overrides parent method).
		
		Args:
			name (str): Name of the source (ex: 'Mocap')
		"""
		super(RealTimePlotWidget, self).addSource(source)
		self.draw()
											
	def appendData(self, variable, source, y):
		"""Adds a data point to the graph (overrides parent method).
			
		No need to provide x coordinate for real ime plot.
			
		Args:
			variable (str): Name of the variable of the point (ex: 'X')
			source (str): Name of the source of the point (ex: 'Mocap')
			y (float): Y value of the data point
		"""
		if self.t_start == None:
			self.t_start = time.time()
			x = 0
		else:
			x = time.time() - self.t_start
		
		# Add datapoint (check also variable and source validity)
		super(RealTimePlotWidget, self).appendData(variable, source, x, y)
		
		# Remove older points if out of sight
		line = self.datapoints[variable][source]
		
		while line[-1][0] - line[0][0] > self.GRAPH_SPAN_X:
			del line[0]
		
	def draw(self):
		"""Redraws the whole graph."""
		
		# Remove all lines from the plot
		for line in self.lines.values():
			line.remove()
			
		self.lines = {}
		
		# Re-create all lines (one per source of current variable)
		if self.variable in self.datapoints.keys():
			
			for source in self.datapoints[self.variable].keys():
				self.lines[source], = self.axes.plot([], [], label=source)
		
		# Setup axis
		self.setupLayout()
		self.resetViewport()
		
	def refresh(self, args):
		"""Animation function to update lines of current variable."""
		
		# Safety
		if not self.variable in self.datapoints.keys():
			return
		
		# Update line data for all sources
		for source in self.lines.keys():
			line = self.datapoints[self.variable][source]
			
			xs = [datapoint[0] for datapoint in line]
			ys = [datapoint[1] for datapoint in line]
			
			self.lines[source].set_xdata(xs)
			self.lines[source].set_ydata(ys)
		
		# Defile plot if needed
		self.resetViewport()
		
		return self.lines,
		
	def resetViewport(self):
		"""Resets the viewport to XY bounds."""
		max_x, max_y = self.getBounds()
		
		# Defile with GRAPH_SPAN_X span
		if max_x > self.GRAPH_SPAN_X:
			self.axes.set_xbound(max_x-self.GRAPH_SPAN_X, max_x)
		else:
			self.axes.set_xbound(0, self.GRAPH_SPAN_X)
		
		# Symetric in Y
		self.axes.set_ybound(-max_y, max_y)
