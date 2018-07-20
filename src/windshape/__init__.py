"""
WindShape master project package.

Used to control a drone using the commander submodule.

Provides also an user interface located in the interface submodule.

The configuration file is located in the common submodule.

Submodules:
	drone: Drone control
	fansarray: Fans array control
	interface: Interface for both controls
"""

from Commander import Commander
from gui.UserInterface import UserInterface
from log.Replay import Replay
