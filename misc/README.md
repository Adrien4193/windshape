# Miscellaneaous files

These files are not part of the ROS package but available for additional informations.

## Alternatives

Alternatives to MAVROS code samples using Dronecore (cpp) and DroneKit (py).

## Clients

OptiTrack clients to collect motion capture data.

### Direct

Direct depacketization in C++ from NatNet 3.0 or older. The .cpp file is all you need to make it work on Visual Studio 2017.

### NatNet

Depacketization using NatNet SDK (32 bits). Includes the SDK header files, the static and dynamic libraries and the source file. Tested on Visual Studio 2017.

You have to put the .dll beside the executable and the .lib beside the source file (.cpp). The include directory must be specified under project properties (C/C++ tab).

### Python

NatNetClient Python class to unpack directly the NatNet 3.0 stream. Run client.py with NatNetClient.py in the same directory.

### VRPN

VRPN C++ client that can re-stream the data in TCP. Same installation as NatNet but without the .dll file.

## Parameters

QGroundControl parameters used for this project (perso.params) and originally set by manufacturer (intel_default.params). CAUTION: The default are for a 3S LiPo battery.

## Server

WindShape fans array Python code that runs on the wind tunnel server (runs main.py).

## Matlab

Simple MATLAB script to see the step response of a low-pass filter as a function of its parameter.
