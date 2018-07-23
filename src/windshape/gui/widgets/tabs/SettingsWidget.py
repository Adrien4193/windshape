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
		self.drone = ComboInput(self, [])
		self.drone.setValue(rospy.get_param('~tracking/tracker'))
		self.layout.addWidget(label, 0, 0)
		self.layout.addWidget(self.drone, 0, 1)
		
		 # Target ID (from VRPN server)
		label = QLabel('Target ID:')
		self.target = ComboInput(self, [])
		self.target.setValue(rospy.get_param('~control/target'))
		self.layout.addWidget(label, 1, 0)
		self.layout.addWidget(self.target, 1, 1)
		
		# Use WS controller
		self.offboard = QCheckBox('Use offboard controller')
		self.offboard.setChecked(rospy.get_param('~control/offboard'))
		self.layout.addWidget(self.offboard, 2, 0)
		
		# Follow target
		self.follow = QCheckBox('Follow target')
		self.follow.setChecked(rospy.get_param('~control/follow'))
		self.layout.addWidget(self.follow, 3, 0)
		
		# Use target attitude as manual input
		self.mimic = QCheckBox('Mimic target')
		self.mimic.setChecked(rospy.get_param('~control/mimic'))
		self.layout.addWidget(self.mimic, 4, 0)
		
		# Drone mask
		mask = rospy.get_param('~control/mask')
		
		self.roll = QCheckBox('Mask Roll')
		self.roll.setChecked(mask[0])
		self.layout.addWidget(self.roll, 5, 0)
		
		self.pitch = QCheckBox('Mask Pitch')
		self.pitch.setChecked(mask[1])
		self.layout.addWidget(self.pitch, 5, 1)
		
		self.yaw = QCheckBox('Mask Yaw')
		self.yaw.setChecked(mask[2])
		self.layout.addWidget(self.yaw, 6, 0)
		
		self.thrust = QCheckBox('Mask Thrust')
		self.thrust.setChecked(mask[3])
		self.layout.addWidget(self.thrust, 6, 1)
		
		# Stretch
		self.layout.setRowStretch(self.layout.rowCount(), 1)
		self.layout.setColumnStretch(self.layout.columnCount(), 1)
		
	def getBodies(self):
		"""Returns current list of bodies (str list)."""
		return self.drone.getItems()
		
	def getControlParameters(self):
		"""Returns drone task and control mode (str)."""
		offboard = self.offboard.isChecked()
		follow = self.follow.isChecked()
		mimic = self.mimic.isChecked()
		mask = [self.roll.isChecked(), self.pitch.isChecked(),
				self.yaw.isChecked(), self.thrust.isChecked()]
		
		return offboard, follow, mimic, mask
		
	def getMocapParameters(self):
		"""Returns drone and target IDs (str)."""
		body = self.drone.getValue()
		target = self.target.getValue()
		
		return body, target
		
	def setBodies(self, bodies):
		"""Updates bodies list.
		
		Args:
			bodies (str list): List of streamed bodies.
		"""
		self.drone.setItems(bodies)
		self.target.setItems(bodies)
