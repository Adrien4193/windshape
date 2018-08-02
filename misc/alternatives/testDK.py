#!/usr/bin/python
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

vehicle = connect('tcp:192.168.8.1:5760', wait_ready=False)
print "Arming motors:"
vehicle.mode    = VehicleMode("MANUAL")
vehicle.armed   = True
while not vehicle.armed:
        print "  Waiting for arming to be finished"
        time.sleep(1)
print "Keeping motors armed for 5s"
time.sleep(5)
print "Disarming"
vehicle.armed   = False