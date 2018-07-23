import os
import math
import time
from threading import Thread, Timer

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

# Main widget
from widgets.ControlPanel import ControlPanel
# Dialog with combo
from dialogs.ComboDialog import ComboDialog

# Replay
from ..log.Replay import Replay

# Drone and fans array control
from ..Commander import Commander


class UserInterface(QMainWindow):
	"""Main window of the interface
	
	Provides GUI for windshape drone control.
	
	Inherits from: QMainWindow.
	
	Overrides: __init__, KeyPressEvent
	"""
	
	# CLASS ATTRIBUTES
	####################################################################
	
	FRAMERATE = 30 # [s]

	# INITIALIZER AND DESTRUCTOR
	####################################################################
 
	def __init__(self):
		"""Initializes parent and starts update."""
		
		super(UserInterface, self).__init__()
		
		# Main class
		self.commander = Commander()
		
		# Drone control
		self.drone = self.commander.getDrone()
		self.control = self.drone.getControlParameters()
		
		# Fans array control
		self.fansArray = self.commander.getFansArray()
		
		# Check drone state
		self.connected = False
		self.armed = False
		self.tracked = False
		self.powered = False
		
		# Setup UI
		self.setupUI()
		
		# Timer updating window
		self.initTimer()
		
		# Shows window
		self.show()
		
	# GUI SETUP
	####################################################################
	
	def setupUI(self):
		"""Creates Graphical User Interface."""
		
		# Window title and dimensions
		self.setWindowTitle('Drone control')
		self.setGeometry(0, 0, 1000, 800)
		
		# Main widget (TabWidget)
		self.controlPanel = ControlPanel(self)
		self.setCentralWidget(self.controlPanel)
		
		# Generates QIcons
		self.createIcons()
		self.setWindowIcon(self.icon_ws)
		
		# Tool bar (top)
		self.toolbar = self.addToolBar('Commands')
		self.initToolBar()
		self.updateToolbar = True  # Updates toolbar actions if True
		
		# Status bar (bottom)
		self.statusbar = self.statusBar()
		self.statusbar.showMessage('Initialization')
		self.updateStatus = True  # Locks status bar if set to False

	def createIcons(self):
		"""Loads QIcons from images in ./icons."""
		path = os.path.dirname(os.path.abspath(__file__))+'/icons'
		
		# WindShape icon (window icon)
		self.icon_ws = QIcon(path + '/windshape.png')
		
		# Drone icons
		self.icon_drone_green = QIcon(path + '/drone_green.png')
		self.icon_drone_red = QIcon(path + '/drone_red.png')
		
		# A, D, M icons (for Arming, Disarming, flight Modes)
		self.icon_A = QIcon(path + '/A.png')
		self.icon_D = QIcon(path + '/D.png')
		self.icon_M = QIcon(path + '/M.png')
		
		# Eyes icons (for tracking state)
		self.icon_opened_eye = QIcon(path + '/opened_eye.png')
		self.icon_closed_eye = QIcon(path + '/closed_eye.png')
		
		# Connection icons (fans array connection)
		self.icon_red = QIcon(path + '/red.png')
		self.icon_green = QIcon(path + '/green.png')
		
		# Start icon (for replay)
		self.icon_start = QIcon(path + '/start.png')
		
		# Top and bottom arrows (for takeoff and land)
		self.icon_top_arrow = QIcon(path + '/top_arrow.png')
		self.icon_bottom_arrow = QIcon(path + '/bottom_arrow.png')
		
	def initToolBar(self):
		"""Loads toolbar actions."""
		
		# Connection
		self.connectAction = QAction(self.icon_drone_red, 'Drone diconnected', self)
		self.connectAction.triggered.connect(self.onConnect)
		self.toolbar.addAction(self.connectAction)
		
		# Fans array PSU
		self.toggleAction = QAction(self.icon_red, 'PSU off', self)
		self.toggleAction.triggered.connect(self.onTogglePSU)
		self.toolbar.addAction(self.toggleAction)
		
		# Tracking
		self.trackAction = QAction(self.icon_closed_eye, 'Drone not tracked', self)
		self.trackAction.triggered.connect(self.onTrack)
		self.toolbar.addAction(self.trackAction)
		
		# Arming
		self.armingAction = QAction(self.icon_D, 'Arm', self)
		self.armingAction.triggered.connect(self.onArm)
		self.toolbar.addAction(self.armingAction)
		
		# Takeoff
		self.takeoffAction = QAction(self.icon_top_arrow, 'Takeoff', self)
		self.takeoffAction.triggered.connect(self.onTakeoff)
		self.toolbar.addAction(self.takeoffAction)
		
		# Land
		self.landAction = QAction(self.icon_bottom_arrow, 'Land', self)
		self.landAction.triggered.connect(self.onLand)
		self.toolbar.addAction(self.landAction)
		
		# Flight mode
		self.modeAction = QAction(self.icon_M, 'Flight mode', self)
		self.modeAction.triggered.connect(self.onMode)
		self.toolbar.addAction(self.modeAction)
		
		# Replay
		self.replayAction = QAction(self.icon_start, 'Replay', self)
		self.replayAction.triggered.connect(self.onReplay)
		self.toolbar.addAction(self.replayAction)
	
	# TOOLBAR AND STATUSBAR FUNCTIONS
	####################################################################
	
	def disableToolBar(self):
		"""Disables all toolbar actions."""
		self.connectAction.setEnabled(False)
		self.trackAction.setEnabled(False)
		self.armingAction.setEnabled(False)
		self.takeoffAction.setEnabled(False)
		self.landAction.setEnabled(False)
		
	def refreshToolBar(self):
		"""Asks for toolbar update."""
		self.updateToolbar = True
		
	def showMessage(self, message, duration=3):
		"""Displays a message for duration beofre overwrite."""
		
		# Locks status bar to block updates and prevents overwriting
		self.updateStatus = False
		
		# Display message
		self.statusbar.showMessage(str(message))
		
		# If duration is None, wait infinite time before unlocking
		if duration is not None:
			Timer(duration, self.unlockStatus).start()
			
	def unlockStatus(self):
		"""Enable overwriting messages in statusbar."""
		self.updateStatus = True
		
	# ACTIONS
	####################################################################
	
	def onConnect(self):
		"""Displays drone status."""
		msg = QMessageBox()
		msg.setIcon(QMessageBox.Information)
		msg.setText(str(self.drone))
		msg.setWindowTitle('Drone information')
		msg.setStandardButtons(QMessageBox.Ok)
		msg.exec_()
		
	def onTrack(self):
		"""Displays tracking status."""
		msg = QMessageBox()
		msg.setIcon(QMessageBox.Information)
		msg.setText(str(self.control.getTarget()))
		msg.setWindowTitle('Target information')
		msg.setStandardButtons(QMessageBox.Ok)
		msg.exec_()
		
	def onArm(self):
		"""Arms or disarm the drone."""
		self.disableToolBar()
		cmd = not self.drone.isArmed()
		
		# Show message
		if cmd:
			self.showMessage('Arming')
		else:
			self.showMessage('Disarming')
			
		# Arm and refresh toolbar
		def arm():
			self.drone.arm(cmd)
			self.updateToolbar = True
			
		Thread(target=arm).start()
		Timer(3, self.updateToolBar).start()
		
	def onTakeoff(self):
		"""Auto-takeoff the drone."""
		self.disableToolBar()
		
		# Takeoff and refresh toolbar
		def takeoff():
			self.drone.takeoff()
			self.updateToolbar = True
		
		# Takeoff in a separate thread
		Thread(target=takeoff).start()
		Timer(3, self.updateToolBar).start()
		
	def onLand(self):
		"""Auto-land the drone."""
		self.disableToolBar()
		
		# Land and refresh toolbar
		def land():
			self.drone.land()
			self.updateToolbar = True
		
		# Land in a separate thread
		Thread(target=land).start()
		Timer(3, self.updateToolBar).start()
	
	def onMode(self):
		"""Changes flight mode with dialog choice."""
		mode = self.drone.getFlightMode()
		modes = self.drone.FLIGHT_MODES
		
		# Flight mode choice
		dialog = ComboDialog(self, 'Flight mode', modes, mode)
		
		# Set new mode in a separate thread if OK pressed
		if dialog.exec_():
			mode = dialog.getResult()
			
			def set_mode():
				self.drone.setFlightMode(mode)
				self.updateToolbar = True
				
			Thread(target=set_mode).start()
			
	def onReplay(self):
		"""Fetches available record files and plot the chosen one."""
		Replay(self)
		
	def onTogglePSU(self):
		"""Turns on or off fans array power supply."""
		self.fansArray.turnOnPSU(not self.fansArray.isPowered())
	
	# UPDATE
	####################################################################
	
	# TIMER
	
	def initTimer(self):
		"""Starts calling update at FRAMERATE."""
		self.timer = QTimer(self)
		self.timer.setInterval(1000.0/UserInterface.FRAMERATE)
		self.timer.timeout.connect(self.update)
		self.timer.start()
	
	# MAIN
	
	def update(self):
		"""Updates toolbar, statusbar and control panel."""
		self.checkStatus()
		
		# Toolbar update if new state or request from a timer
		if self.updateToolbar:
			self.updateToolBar()
			self.updateStatusBar()
			self.updateToolbar = False
		
		# Show drone status if no message to display
		if self.updateStatus:
			self.updateStatusBar()
		
		# Update tabs
		self.updateWind()
		self.updateInfo()
		self.updatePlot()
		self.updateSettings()
		
	# DRONE STATUS
	
	def checkStatus(self):
		"""Asks for refresh if something needs display."""
		
		if self.connected != self.drone.isConnected():
			self.connected = self.drone.isConnected()
			self.updateToolbar = True
		
		if self.armed != self.drone.isArmed():
			self.armed = self.drone.isArmed()
			self.updateToolbar = True
		
		if self.tracked != self.drone.isTracked():
			self.tracked = self.drone.isTracked()
			self.updateToolbar = True
			
		if self.powered != self.fansArray.isPowered():
			self.powered = self.fansArray.isPowered()
			self.updateToolbar = True
	
	# STATUS BAR
	
	def updateStatusBar(self):
		"""Updates the status bar message."""
		
		if self.drone.isConnected():
			message = 'Connected'
		else:
			message = 'Disconnected'
			
		if self.drone.isArmed():
			message += ', Armed'
			
		self.statusbar.showMessage(message)
		self.updateStatus = True
		
	# TOOL BAR

	def updateToolBar(self):
		"""Updates the tool bar icons and activation."""
		self.disableToolBar()
		
		# Checks which actions are permitted and updates icons
		self.updateConnectAction()
		self.updateTrackAction()
		self.updateArmingAction()
		self.updateTLAction()
		self.updateToggleAction()
		
	def updateConnectAction(self):
		"""Updates connection icon."""
		
		# Icon and tool tip
		if self.drone.isConnected():
			self.connectAction.setIcon(self.icon_drone_green)
			self.connectAction.setToolTip('Drone connected')
		else:
			self.connectAction.setIcon(self.icon_drone_red)
			self.connectAction.setToolTip('Drone disconnected')
		
		# Always enabled
		self.connectAction.setEnabled(True)
		
	def updateToggleAction(self):
		"""Change fans array power supply icon."""
		
		if self.fansArray.isPowered():
			self.toggleAction.setIcon(self.icon_green)
			self.toggleAction.setToolTip('PSU on')
		else:
			self.toggleAction.setIcon(self.icon_red)
			self.toggleAction.setToolTip('PSU off')
		
	def updateTrackAction(self):
		"""Updates tracking icon."""
		
		# Icon and tool tip
		if self.drone.isTracked():
			self.trackAction.setIcon(self.icon_opened_eye)
			self.trackAction.setToolTip('Drone tracked')
		else:
			self.trackAction.setIcon(self.icon_closed_eye)
			self.trackAction.setToolTip('Drone not tracked')
		
		# Always enabled
		self.trackAction.setEnabled(True)
		
	def updateArmingAction(self):
		"""Enables/disables arming action and updates icon."""
		
		# Arming icon = A if armed
		if self.drone.isArmed():
			self.armingAction.setIcon(self.icon_A)
			self.armingAction.setToolTip('Disarm')
			
		# Arming icon = D if disarmed
		else:
			self.armingAction.setIcon(self.icon_D)
			self.armingAction.setToolTip('Arm')
		
		# Can arm/disarm only if drone connected
		if self.drone.isConnected():
			self.armingAction.setEnabled(True)
			
	def updateTLAction(self):
		"""Enables/disables takeoff/land action."""
		
		# Can takeoff/land only if armed
		if self.drone.isArmed():
			self.takeoffAction.setEnabled(True)
			self.landAction.setEnabled(True)
	
	# WIDGETS
	
	def updateWind(self):
		"""Updates wind tab (activation of auto wind)."""
		activate = self.controlPanel.getAutoWind()
		self.commander.setAutoWind(activate)
	
	def updateInfo(self):
		"""Updates info tab (get data from drone)."""
		
		# Get info source from CP
		source = self.controlPanel.getSource()
		
		if source == 'Info':
			text = self.commander.getStatus()
			
		elif source == 'Pose':
			text = self.commander.getState()
					
		elif source == 'Command':
			text = self.commander.getCommand()
		
		# Send info to CP
		self.controlPanel.displayInfo(text)
		
	def updatePlot(self):
		"""Plots drone pose (mocap, FCU, setpoint)."""
		mocap = self.drone.getMocapPose()
		estimated = self.drone.getEstimatedPose()
		setpoint = self.drone.getControlParameters().getSetpoint()
		
		# Send to control panel as list with source label
		self.controlPanel.plotData('Mocap', list(mocap))
		self.controlPanel.plotData('FCU', list(estimated))
		self.controlPanel.plotData('Setpoint', list(setpoint))
		
	def updateSettings(self):
		"""Updates drone from settings tab."""
		body, target = self.controlPanel.getMocapParameters()
		
		if body != self.drone.getMocapLabel():
			self.drone.setMocapLabel(body)
			self.showMessage('Drone tracker set to '+str(body))
			
		if target != self.control.getTarget().getLabel():
			self.control.setTarget(target)
			self.showMessage('Drone target set to '+str(target))
		
		offboard, follow, mimic, mask = self.controlPanel.getSettings()
		
		if offboard != self.control.isUsingOffboardControl():
			self.control.useOffboardControl(offboard)
			self.showMessage('Offboard control set to '+str(offboard))
		
		if follow != self.control.isFollowingTarget():
			self.control.followTarget(follow)
			self.showMessage('Target following set to '+str(follow))
			
		if mimic != self.control.isMimingTarget():
			self.control.mimicTarget(mimic)
			self.showMessage('Target imitation set to '+str(mimic))
			
		if mask != self.control.getMask():
			self.control.setMask(*mask)
			self.showMessage('Control mask set to '+str(mask))
			
		# Update drone manual inputs
		roll, pitch, yaw, thrust = self.controlPanel.getAttitude()
		self.control.setManualAttitude(roll, pitch, yaw, thrust)
		
		# Update rigid bodies list in CP
		self.controlPanel.updateBodies(self.drone.getTrackersList())

	# KEYBOARD EVENT
	####################################################################
	
	def keyPressEvent(self, event):
		"""Press Enter to send setpoint, PWM or validate settings."""
		
		# Enter pressed
		if event.key() == Qt.Key_Return or event.key() == Qt.Key_Enter:
			
			# Setpoint tab active
			if self.controlPanel.currentIndex() == 0:
				self.sendSetpoint()
				
			# PWM tab active
			elif self.controlPanel.currentIndex() == 2:
				self.sendPWM()
				
	def sendSetpoint(self):
		"""Sends setpoint to the drone chosen in control panel."""

		if self.controlPanel.setpointValid():
			x, y, z, yaw = self.controlPanel.getSetpoint()
			self.showMessage('Pose sent: '+str((x, y, z, yaw)))
			self.drone.setManualSetpoint(x, y, z, yaw)
		else:
			self.showMessage('Invalid setpoint')
														
	def sendPWM(self):
		"""Sends PWM value to fans array chosen in control panel."""

		if self.controlPanel.pwmValid():
			pwm = self.controlPanel.getPWM()
			self.showMessage('PWM sent: '+str(pwm))
			self.fansArray.setPWM(pwm)
		else:
			self.showMessage('Invalid PWM')
