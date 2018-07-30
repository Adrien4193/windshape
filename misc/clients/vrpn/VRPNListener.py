#!/usr/bin/env python

import sys
import time
import socket

# Bonus for conversion
# Source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
def quaternion_to_euler(x, y, z, w):
	
	t0 = 2.0 * (w*x + y*z)
	t1 = 1.0 - 2.0 * (x*x + y*y)
	roll = math.degrees(math.atan2(t0, t1))
	
	t2 = 2.0 * (w*y - z*x)
	t2 = 1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	pitch = math.degrees(math.asin(t2))
	
	t3 = +2.0 * (w*z + x*y)
	t4 = +1.0 - 2.0 * (y*y + z*z)
	yaw = math.degrees(math.atan2(t3, t4))
	
	return roll, pitch, yaw

TCP_IP = '127.0.0.1'
TCP_PORT = 9999
BUFFER_SIZE = 1024

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

start = time.time()

while True:

    try:
        data = s.recv(BUFFER_SIZE)
    except socket.error as e:
        print e
        break
    
    print data

s.close()