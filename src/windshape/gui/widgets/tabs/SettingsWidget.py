from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

# ROS main library
import rospy

# Personal libraries
from ..inputs.ComboInput import ComboInput
from ..inputs.EditInput import EditInput


class SettingsWidget(QWidget):
	"""Widget composed of EditInput widgets.
	
	Inherits from QWidget.
	
	Overrides: __init__
	"""
	
	def __init__(self, parent):
		"""Creates inputs.
		
		Args:
			parent (QWidget): The parent widget to insert instance
		"""
		super(SettingsWidget, self).__init__(parent)
		
		# GridLayout for main layout
		self.layout = QGridLayout()
		self.setLayout(self.layout)
		
		# Drone ID (from VRPN server)
		label = QLabel('Drone ID:')
		self.body = ComboInput(self, [])
		self.body.setValue(rospy.get_param('~tracking/tracker_drone'))
		self.layout.addWidget(label, 0, 0)
		self.layout.addWidget(self.body, 0, 1)
		
		 # Target ID (from VRPN server)
		label = QLabel('Target ID:')
		self.target = ComboInput(self, [])
		self.target.setValue(rospy.get_param('~tracking/tracker_target'))
		self.layout.addWidget(label, 1, 0)
		self.layout.addWidget(self.target, 1, 1)
		
		# Drone task
		label = QLabel('Task:')
		self.task = ComboInput(self, ['follow_target', 'reach_setpoint'])
		self.task.setValue(rospy.get_param('~control/task'))
		self.layout.addWidget(label, 2, 0)
		self.layout.addWidget(self.task, 2, 1)
		
		# Drone control mode
		label = QLabel('Control mode:')
		self.mode = ComboInput(self, ['onboard', 'offboard'])
		self.mode.setValue(rospy.get_param('~control/mode'))
		self.layout.addWidget(label, 3, 0)
		self.layout.addWidget(self.mode, 3, 1)
		
		# Drone mask
		mask = rospy.get_param('~control/mask')
		
		self.roll = QCheckBox('Mask Roll')
		self.roll.setChecked(mask[0])
		self.layout.addWidget(self.roll, 4, 0)
		
		self.pitch = QCheckBox('Mask Pitch')
		self.pitch.setChecked(mask[1])
		self.layout.addWidget(self.pitch, 4, 1)
		
		self.yaw = QCheckBox('Mask Yaw')
		self.yaw.setChecked(mask[2])
		self.layout.addWidget(self.yaw, 5, 0)
		
		self.thrust = QCheckBox('Mask Thrust')
		self.thrust.setChecked(mask[3])
		self.layout.addWidget(self.thrust, 5, 1)
		
		# Stretch
		self.layout.setRowStretch(self.layout.rowCount(), 1)
		self.layout.setColumnStretch(self.layout.columnCount(), 1)
		
	def getBodies(self):
		"""Returns current list of bodies (str list)."""
		return self.body.getItems()
		
	def getControlParameters(self):
		"""Returns drone task and control mode (str)."""
		task = self.task.getValue()
		mode = self.mode.getValue()
		mask = [self.roll.isChecked(), self.pitch.isChecked(),
				self.yaw.isChecked(), self.thrust.isChecked()]
		
		return task, mode, mask
		
	def getMocapParameters(self):
		"""Returns VRPN server IP, drone and target IDs (str)."""
		body = self.body.getValue()
		target = self.target.getValue()
		
		return body, target
		
	def setBodies(self, bodies):
		"""Updates bodies list.
		
		Args:
			bodies (str list): List of streamed bodies.
		"""
		self.body.setItems(bodies)
		self.target.setItems(bodies)
