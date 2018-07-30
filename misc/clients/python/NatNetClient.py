import sys
import time
import socket
import struct
import timecode
from threading import Thread

def trace(*args):
    pass#' '.join([str(arg) for arg in args])

# Create structs for reading various object types to speed up parsing.
ShortValue = struct.Struct('<h')
IntegerValue = struct.Struct('<i')
UnsignedIntegerValue = struct.Struct('<I')
UnsignedLongValue = struct.Struct('<Q')
FloatValue = struct.Struct('<f')
DoubleValue = struct.Struct('<d')
Vector3 = struct.Struct('<fff')
Quaternion = struct.Struct('<ffff')

class NatNetClient:

    def __init__(self, serverIP):

        # Change this value to the IP address of the NatNet server.
        self.serverIPAddress = serverIP

        # This should match the multicast address listed in Motive's streaming settings.
        self.multicastAddress = "239.255.42.99"

        # NatNet Command channel
        self.commandPort = 1510
        
        # NatNet Data channel     
        self.dataPort = 1511

        # Set this to a callback method of your choice to receive per-rigid-body data at each frame.
        self.rigidBodyListener = None
        self.newFrameListener = None
        
        # NatNet stream version. This will be updated to the actual version the server is using during initialization.
        self.__natNetStreamVersion = (3,0,0,0)

        # Conversion from ID to name
        self.__names = {}

    # Client/server message ids
    NAT_PING                  = 0 
    NAT_PINGRESPONSE          = 1
    NAT_REQUEST               = 2
    NAT_RESPONSE              = 3
    NAT_REQUEST_MODELDEF      = 4
    NAT_MODELDEF              = 5
    NAT_REQUEST_FRAMEOFDATA   = 6
    NAT_FRAMEOFDATA           = 7
    NAT_MESSAGESTRING         = 8
    NAT_DISCONNECT            = 9
    NAT_UNRECOGNIZED_REQUEST  = 100

    # Assets ids
    NAT_MARKERSET   = 0
    NAT_RIGIDBODY   = 1
    NAT_SKELETON    = 2

##################################################################################################
# UDP SOCKETS
##################################################################################################

    # Create a data socket to attach to the NatNet stream (get data from Motive)
    def __createDataSocket(self, port):
        result = socket.socket(socket.AF_INET,     # Internet
                              socket.SOCK_DGRAM,
                              socket.IPPROTO_UDP)    # UDP
        result.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        result.bind(('', port))

        mreq = struct.pack("4sl", socket.inet_aton(self.multicastAddress), socket.INADDR_ANY)
        result.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        return result

    # Create a command socket to attach to the NatNet stream (ask for data to Motive)
    def __createCommandSocket(self):
        result = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        result.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        result.bind(('', 0))
        result.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        return result

#####################################################################################################
# ASSETS (RIGID BODIES, SKELETONS)
#####################################################################################################

    # Unpack a rigid body object from a data packet
    def __unpackRigidBody(self, data):
        offset = 0

        # ID (4 bytes)
        id_, = IntegerValue.unpack(data[offset:offset+4])
        offset += 4
        trace("ID:", id_)

        # Position and orientation
        pos = Vector3.unpack(data[offset:offset+12])
        offset += 12
        trace("\tPosition: {:.3f} {:.3f} {:.3f}".format(pos[0], pos[1], pos[2]))
        rot = Quaternion.unpack(data[offset:offset+16])
        offset += 16
        trace("\tOrientation: {:.3f} {:.3f} {:.3f} {:.3f}".format(rot[0], rot[1], rot[2], rot[3]))

        # Mean markers error
        if (self.__natNetStreamVersion[0] >= 2):
            markerError, = FloatValue.unpack(data[offset:offset+4])
            offset += 4
            trace("\tMarker Error:", markerError)

        # Version 2.6 and later
        if (((self.__natNetStreamVersion[0] == 2) and (self.__natNetStreamVersion[1] >= 6)) or self.__natNetStreamVersion[0] > 2 or self.__natNetStreamVersion[0] == 0):
            param, = ShortValue.unpack(data[offset:offset+2])
            trackingValid = (param & 0x01) != 0
            offset += 2
            trace("\tTracking Valid:", trackingValid)

        # Send information to any listener.
        if self.rigidBodyListener is not None:
            name = self.__names[id_] if id_ in self.__names.keys() else "Unknown"
            self.rigidBodyListener(id_, name, trackingValid, pos, rot)

        return offset

    # Unpack a skeleton object from a data packet
    def __unpackSkeleton(self, data):
        offset = 0
        
        # ID
        id_, = IntegerValue.unpack(data[offset:offset+4])
        offset += 4
        trace("ID:", id_)
        
        # Bones number
        rigidBodyCount, = IntegerValue.unpack(data[offset:offset+4])
        offset += 4
        trace("Rigid Body Count:", rigidBodyCount)

        # Extract each bone as a rigid body
        for j in range(0, rigidBodyCount):
            offset += self.__unpackRigidBody(data[offset:])

        return offset

#############################################################################################################
# MOCAP DATA (MARKERS, VERSION, TIME STAMP, ...)
#############################################################################################################

    # Unpack data from a motion capture frame message
    def __unpackMocapData(self, data):
        trace("Begin MoCap Frame\n-----------------")

        offset = 0
        
        # Frame number (4 bytes)
        frameNumber, = IntegerValue.unpack(data[offset:offset+4])
        offset += 4
        trace("Frame #:", frameNumber)

        # Marker set count (4 bytes)
        markerSetCount, = IntegerValue.unpack(data[offset:offset+4])
        offset += 4
        trace("Marker Set Count:", markerSetCount)

        # Loop over marker sets
        for i in range(0, markerSetCount):

            # Model name
            modelName, separator, remainder = bytes(data[offset:]).partition(b'\0')
            offset += len(modelName) + 1
            trace("Model Name:", modelName.decode('utf-8'))

            # Marker count (4 bytes)
            markerCount, = IntegerValue.unpack(data[offset:offset+4])
            offset += 4
            trace("Marker Count:", markerCount)

            for j in range(0, markerCount):
                pos = Vector3.unpack(data[offset:offset+12])
                offset += 12
                trace("\tMarker {}: {:.3f}, {:.3f}, {:.3f}".format(j, pos[0], pos[1], pos[2]))
        
        ##################################
        # UNLABELED MARKERS (DEPRECATED) #
        ##################################

        # Unlabeled markers count (4 bytes)
        unlabeledMarkersCount, = IntegerValue.unpack(data[offset:offset+4])
        offset += 4
        #trace("Unlabeled Markers Count:", unlabeledMarkersCount)

        for i in range(0, unlabeledMarkersCount):
            pos = Vector3.unpack(data[offset:offset+12])
            offset += 12
            #trace("\tMarker {}: {:.3f}, {:.3f}, {:.3f}".format(j, pos[0], pos[1], pos[2]))

        ################
        # RIGID BODIES #
        ################

        # Rigid body count (4 bytes)
        rigidBodyCount, = IntegerValue.unpack(data[offset:offset+4])
        offset += 4
        trace("Rigid Body Count:", rigidBodyCount)

        for i in range(0, rigidBodyCount):
            offset += self.__unpackRigidBody(data[offset:])

        #############
        # SKELETONS #
        #############

        skeletonCount = 0

         # Version 2.1 and later
        if ((self.__natNetStreamVersion[0] == 2 and self.__natNetStreamVersion[1] > 0) or self.__natNetStreamVersion[0] > 2):
            skeletonCount, = IntegerValue.unpack(data[offset:offset+4])
            offset += 4
            trace("Skeleton Count:", skeletonCount)

            for i in range(0, skeletonCount):
                offset += self.__unpackSkeleton(data[offset:])

        ####################
        # LABELLED MARKERS #
        ####################

        labeledMarkerCount = 0

        # Version 2.3 and later
        if ((self.__natNetStreamVersion[0] == 2 and self.__natNetStreamVersion[1] > 3) or self.__natNetStreamVersion[0] > 2):
            labeledMarkerCount, = IntegerValue.unpack(data[offset:offset+4])
            offset += 4
            trace("Labeled Marker Count:", labeledMarkerCount)

            for i in range(0, labeledMarkerCount):
                markerID, = ShortValue.unpack(data[offset:offset+2])
                modelID, = ShortValue.unpack(data[offset+2:offset+4])
                offset += 4
                pos = Vector3.unpack(data[offset:offset+12])
                offset += 12
                size, = FloatValue.unpack(data[offset:offset+4])
                offset += 4
                trace("\tID: [MarkerID {}] [ModelID {}]".format(markerID, modelID))
                trace("\tPosition: {:.3f} {:.3f} {:.3f}".format(pos[0], pos[1], pos[2]))
                trace("\tSize: {:.3f})".format(size))

                # Version 2.6 and later
                if ((self.__natNetStreamVersion[0] == 2 and self.__natNetStreamVersion[1] >= 6) or self.__natNetStreamVersion[0] > 2):
                    params, = ShortValue.unpack(data[offset:offset+2])
                    offset += 2
                    occluded = (params & 0x01) != 0
                    pointCloudSolved = (params & 0x02) != 0
                    modelSolved = (params & 0x04) != 0
                    trace("\tSolved:", occluded, pointCloudSolved, modelSolved)

                # Version 3.0 and later
                if (self.__natNetStreamVersion[0] >= 3):
                    hasModel = (params & 0x08) != 0
                    unlabeled = (params & 0x10) != 0
                    active = (params & 0x20) != 0
                    trace("\tModel:", hasModel, unlabeled, active)

                # Version 3.0 and later
                if (self.__natNetStreamVersion[0] >= 3):
                    residual, = FloatValue.unpack(data[offset:offset+4])
                    offset += 4
                    trace("\tResidual:", residual)

        ####################
        # FORCE PLATE DATA #
        ####################

        # Force Plate data (version 2.9 and later)
        if ((self.__natNetStreamVersion[0] == 2 and self.__natNetStreamVersion[1] >= 9) or self.__natNetStreamVersion[0] > 2):
            forcePlateCount, = IntegerValue.unpack(data[offset:offset+4])
            offset += 4
            trace("Force Plate Count:", forcePlateCount)

            for i in range(0, forcePlateCount):

                # ID
                forcePlateID, = IntegerValue.unpack(data[offset:offset+4])
                offset += 4
                trace("Force Plate {}: {}".format(i, forcePlateID))

                # Channel Count
                forcePlateChannelCount, = IntegerValue.unpack(data[offset:offset+4])
                offset += 4

                # Channel Data
                for j in range(0, forcePlateChannelCount):
                    trace("\tChannel {}: {}".format(j, forcePlateID))
                    forcePlateChannelFrameCount, = IntegerValue.unpack(data[offset:offset+4])
                    offset += 4

                    for k in range(0, forcePlateChannelFrameCount):
                        forcePlateChannelVal, = IntegerValue.unpack(data[offset:offset+4])
                        offset += 4
                        trace("\t\t", forcePlateChannelVal)

        ###############
        # DEVICE DATA #
        ###############

        # Device data (version 2.11 and later)
        if ((self.__natNetStreamVersion[0] == 2 and self.__natNetStreamVersion[1] >= 11) or self.__natNetStreamVersion[0] > 2):
            deviceCount, = IntegerValue.unpack(data[offset:offset+4])
            offset += 4
            trace("Device Count:", deviceCount)

            for i in range(0, deviceCount):

                # ID
                deviceID, = IntegerValue.unpack(data[offset:offset+4])
                offset += 4
                trace("Device {}: {}".format(i, deviceID))

                # Channel Count
                deviceChannelCount, = IntegerValue.unpack(data[offset:offset+4])
                offset += 4

                # Channel Data
                for j in range(0, deviceChannelCount):
                    trace("\tChannel {}: {}", j, deviceID)
                    deviceChannelFrameCount, = IntegerValue.unpack(data[offset:offset+4])
                    offset += 4

                    for k in range(0, deviceChannelFrameCount):
                        deviceChannelVal, = IntegerValue.unpack(data[offset:offset+4])
                        offset += 4
                        trace("\t\t", deviceChannelVal)
		
        ##############
        # PARAMETERS #
        ##############

        if (self.__natNetStreamVersion[0] < 3):
            softwareLatency, = FloatValue.unpack(data[offset:offset+4])
            offset += 4
            trace("Software latency:", softwareLatency)

        # Timecode            
        tc, = UnsignedIntegerValue.unpack(data[offset:offset+4])
        offset += 4
        sub, = UnsignedIntegerValue.unpack(data[offset:offset+4])
        offset += 4
        trace("Timecode:", tc, sub)

        # Timestamp (increased to double precision in 2.7 and later)
        if ((self.__natNetStreamVersion[0] == 2 and self.__natNetStreamVersion[1] >= 7) or self.__natNetStreamVersion[0] > 2):
            timestamp, = DoubleValue.unpack(data[offset:offset+8])
            offset += 8
        else:
            timestamp, = FloatValue.unpack(data[offset:offset+4])
            offset += 4
        
        trace("Timestamp:", timestamp)

        # High res Timestamp (Version 3.0 and later)
        if ((self.__natNetStreamVersion[0] >= 3)):
            stampCameraExposure, = UnsignedLongValue.unpack(data[offset:offset+8])
            offset += 8
            stampDataReceived, = UnsignedLongValue.unpack(data[offset:offset+8])
            offset += 8
            stampTransmit, = UnsignedLongValue.unpack(data[offset:offset+8])
            offset += 8
            trace("Mid-exposure timestamp:", stampCameraExposure)
            trace("Camera data received timestamp :", stampDataReceived)
            trace("Transmit timestamp :", stampTransmit)

        # Frame parameters
        param, = ShortValue.unpack(data[offset:offset+2])
        isRecording = (param & 0x01) != 0
        trackedModelsChanged = (param & 0x02) != 0
        offset += 2
        trace("Frame parameters:", isRecording, trackedModelsChanged)

        # End of data tag
        eod, = IntegerValue.unpack(data[offset:offset+4])
        trace("-----------------\nEnd MoCap Frame")

        # Send information to any listener.
        if self.newFrameListener is not None:
            self.newFrameListener(frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                                  labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged)

##############################################################################################################################
# ASSETS DESCRIPTIONS
##############################################################################################################################

    # Unpack a marker set description packet
    def __unpackMarkerSetDescription(self, data):
        offset = 0

        # Market set info
        name, separator, remainder = bytes(data[offset:]).partition(b'\0')
        offset += len(name) + 1
        trace("\tName:", name.decode('utf-8'))
        
        # Number of markers
        markerCount, = IntegerValue.unpack(data[offset:offset+4])
        offset += 4

        # Unpack each marker
        for i in range(0, markerCount):
            name, separator, remainder = bytes(data[offset:]).partition(b'\0')
            offset += len(name) + 1
            trace("\tMarker Name:", name.decode('utf-8'))
        
        return offset

    # Unpack a rigid body description packet
    def __unpackRigidBodyDescription(self, data):
        offset = 0

        # Body label (Version 2.0 or higher)
        if self.__natNetStreamVersion[0] >= 2:
            name, separator, remainder = bytes(data[offset:]).partition(b'\0')
            offset += len(name) + 1
            trace("\tName:", name.decode('utf-8'))

        id_, = IntegerValue.unpack(data[offset:offset+4])
        trace("\tID:", id_)
        offset += 4

        # Update correspondance
        self.__names[id_] = name

        # Parent ID
        parentID, = IntegerValue.unpack(data[offset:offset+4])
        trace("\tParent ID:", parentID)
        offset += 4

        # Time stamp
        timestamp = Vector3.unpack(data[offset:offset+12])
        trace("\tTime stamp:", timestamp)
        offset += 12

        # Per-marker data (Version 3.0 or higher)
        if self.__natNetStreamVersion[0] >= 3:

            # Number of markers
            nMarkers, = IntegerValue.unpack(data[offset:offset+4])
            offset += 4

            # Markers position (3 floats / marker)
            positions = []

            for i in range(nMarkers):
                position = Vector3.unpack(data[offset:offset+12])
                positions.append(position)
                offset += 12

            # Marker required label (1 int / label)
            required_labels = []

            for i in range(nMarkers):
                required_label, = IntegerValue.unpack(data[offset:offset+4])
                required_labels.append(required_label)
                offset += 4

            # Display
            trace('\tMarkers:')

            for i in range(nMarkers):
                x, y, z = positions[i]
                label = required_labels[i]
                trace('\t\t{:.2f}, {:.2f}, {:.2f} ({})'.format(x, y, z, label))
        
        return offset

    # Unpack a skeleton description packet
    def __unpackSkeletonDescription(self, data):
        offset = 0

        # Info
        name, separator, remainder = bytes(data[offset:]).partition(b'\0')
        offset += len(name) + 1
        trace("\tName:", name.decode('utf-8'))
        
        # ID
        id_, = IntegerValue.unpack(data[offset:offset+4])
        trace("\tID:", id_)
        offset += 4

        # Number of bones
        rigidBodyCount, = IntegerValue.unpack(data[offset:offset+4])
        trace("\tNumber of bones:", rigidBodyCount)
        offset += 4

        # Unpack bones as rigid bodies
        for i in range(0, rigidBodyCount):
            offset += self.__unpackRigidBodyDescription(data[offset:])

        return offset

    # Unpack a data description packet
    def __unpackDataDescriptions(self, data):
        offset = 0

        # Number of assets
        datasetCount, = IntegerValue.unpack(data[offset:offset+4])
        trace('dataset count:', datasetCount)
        offset += 4

        # Reset correspondance
        self.__names = {}

        # Extract assets
        for i in range(0, datasetCount):

            # Asset type ((marker=0, rigid bodies=1, skeletons=2)
            asset, = IntegerValue.unpack(data[offset:offset+4])
            offset += 4

            # Unpack asset
            if asset == self.NAT_MARKERSET:
                trace('Asset ', str(i)+': Marker Set')
                offset += self.__unpackMarkerSetDescription(data[offset:])
            elif asset == self.NAT_RIGIDBODY:
                trace('Asset ', str(i)+': Rigid Body')
                offset += self.__unpackRigidBodyDescription(data[offset:])
            elif asset == self.NAT_SKELETON:
                trace('Asset ', str(i)+': Skeleton')
                offset += self.__unpackSkeletonDescription(data[offset:])

##################################################################################################
# MAIN LOOP
##################################################################################################

    def __dataThreadFunction(self, socket):

        while True:
            data, addr = socket.recvfrom(32768)

            if (len(data) > 0):
                self.__processMessage(data)

    def __processMessage(self, data):
        trace("Begin Packet\n------------")

        #
        # MESSAGE STRUCTURE: ID (2 bytes) SIZE (2 bytes) DATA (size bytes)
        #
        # See __init__ for messages IDs, DATA is the message content
        #

        # Message type
        messageID, = ShortValue.unpack(data[0:2])
        trace("Message ID: ", messageID)
        
        # Message size
        packetSize, = ShortValue.unpack(data[2:4])
        trace("Packet Size: ", packetSize)

        offset = 4

        # Unpack message
        if (messageID == self.NAT_FRAMEOFDATA):
            self.__unpackMocapData(data[offset:])

        elif (messageID == self.NAT_MODELDEF):
            self.__unpackDataDescriptions(data[offset:])

        elif (messageID == self.NAT_PINGRESPONSE):
            offset += 256   # Skip the sending app's Name field
            offset += 4     # Skip the sending app's Version info
            self.__natNetStreamVersion = struct.unpack('BBBB', data[offset:offset+4])
            trace('NatNet version: ', self.__natNetStreamVersion)
            offset += 4

        elif (messageID == self.NAT_RESPONSE):

            if (packetSize == 4):
                commandResponse, = IntegerValue.unpack(data[offset:offset+4])
                offset += 4
                trace("Command response:", commandResponse)
            else:
                message, separator, remainder = bytes(data[offset:]).partition(b'\0')
                offset += len(message) + 1
                trace("Command response:", message.decode('utf-8'))

        elif (messageID == self.NAT_UNRECOGNIZED_REQUEST):
            trace("Received 'Unrecognized request' from server")

        elif (messageID == self.NAT_MESSAGESTRING):
            message, separator, remainder = bytes(data[offset:]).partition(b'\0')
            offset += len(message) + 1
            trace("Received message from server:", message.decode('utf-8'))

        else:
            trace("ERROR: Unrecognized packet type")
            
        trace("----------\nEnd Packet\n")

##############################################################################################
# SERVER REQUESTS
##############################################################################################
    
    # Send command to server
    def sendCommand(self, command, commandStr, socket, address):

        # Auto-fill commandStr for known commands
        if (command == self.NAT_REQUEST_MODELDEF or command == self.NAT_REQUEST_FRAMEOFDATA):
            packetSize = 0
            commandStr = ""
        elif (command == self.NAT_REQUEST):
            packetSize = len(commandStr) + 1
        elif (command == self.NAT_PING):
            commandStr = "Ping"
            packetSize = len(commandStr) + 1

        # Pack command and packet size as header
        data = IntegerValue.pack(command)
        data += IntegerValue.pack(packetSize)
        
        # Choose UTF-8 encoding and add carriage return
        data += commandStr.encode('utf-8')
        data += b'\0'

        # Send command over command port
        socket.sendto(data, address)

    # Return NatNetVersion
    def getVersion():
        return self.__natNetStreamVersion

##################################################################################################
# INIT
##################################################################################################    

    def run(self):
        # Create the data socket
        self.dataSocket = self.__createDataSocket(self.dataPort)
        if (self.dataSocket is None):
            print("Could not open data channel")
            exit

        # Create the command socket
        self.commandSocket = self.__createCommandSocket()
        if (self.commandSocket is None):
            print("Could not open command channel")
            exit

        # Create a separate thread for receiving data packets
        dataThread = Thread(target = self.__dataThreadFunction, args = (self.dataSocket,))
        #dataThread.setDaemon(True)
        dataThread.start()

        # Create a separate thread for receiving command packets
        commandThread = Thread(target = self.__dataThreadFunction, args = (self.commandSocket,))
        #commandThread.setDaemon(True)
        commandThread.start()

        # Ping server to get NatNet version
        self.sendCommand(self.NAT_PING, "", self.commandSocket, (self.serverIPAddress, self.commandPort))

        # Request model definition (correspondance label <-> id)
        self.sendCommand(self.NAT_REQUEST_MODELDEF, "", self.commandSocket, (self.serverIPAddress, self.commandPort))

################################################################################################
# TEST
################################################################################################

if __name__ == "__main__":

    # This will create a new NatNet client
    streamingClient = NatNetClient("127.0.0.1")

    # Start up the streaming client
    streamingClient.run()

    # Infinite loop
    while True:
        time.sleep(1)
