import math
import time

from NatNetClient import NatNetClient

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

# Callback called at each RB of each frame
def rigid_body_cb(id_, label, tracked, pos, ori):

    print "RigidBody {} ({}):".format(label, "tracked" if tracked else "untracked")
    print "    ID:", id_
    print "    Position [mm]: x:{:.2f}, y:{:.2f}, z:{:.2f}".format(1000.0*pos[0], 1000.0*pos[1], 1000.0*pos[2])
    #print "    Orientation [quaternion]: x:{:.4f}, y:{:.4f}, z:{:.4f}, w:{:.4f}".format(ori[0], ori[1], ori[2], ori[3])
    print "    Orientation [deg]: roll:{:.2f}, pitch:{:.2f}, yaw:{:.2f}".format(*quaternion_to_euler(*ori))

# This will create a new NatNet client
streamingClient = NatNetClient("127.0.0.1")

# Set a callback to get each RB info in real time
streamingClient.rigidBodyListener = rigid_body_cb

# Start up the streaming client
streamingClient.run()

# Infinite loop
while True:
    time.sleep(1)