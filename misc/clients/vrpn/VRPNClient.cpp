#include "vrpn_Tracker.h"
#include "vrpn_Button.h"
#include "vrpn_Analog.h"

#include "vrpn_Text.h"
#include "vrpn_Connection.h"

#include <iostream>
#include <math.h>
#include <conio.h>

using namespace std;

//////////////////////////////////////////////////////////////////////////////
// PARAMETERS
///////////////////////////////////////////////////////////////////////////////

// VRPN rigid body parameters
string probe_name = "drone"; // Label (on Motive)
string vrpn_server_ip = "localhost"; // VRPN server ip (computer running Motive)

// TCP stream
bool stream = false; // Stream if true
int port = 9999; // Communication port

//////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES
//////////////////////////////////////////////////////////////////////////////

// Connection status with client
bool client_connected = false;

// VRPN data
vrpn_Tracker_Remote * vrpnTracker; // VRPN tracker
double probe_pos[3]; // Position (x, y, z)
double probe_quat[4]; // Orientation (x, y, z, w)

// TCP communication
WSADATA WSAData; // Windows networking interface

SOCKADDR_IN addr; // Server socket parameters
SOCKET server; // Server socket

SOCKADDR_IN from; // Client socket parameters
SOCKET client; // Client socket

// VRPN server

// Custom Analog
class myAnalog : public vrpn_Analog
{
public:
	myAnalog(vrpn_Connection *c = 0);
	virtual ~myAnalog() {};

	virtual void mainloop();

protected:
	struct timeval _timestamp;
};

// Analog instance
myAnalog * serverAnalog;

// Connection to VRPN manager
vrpn_Connection_IP * myConnection;

//////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
//////////////////////////////////////////////////////////////////////////////

// VRPN callback called when data is available
void VRPN_CALLBACK handle_tracker(void* userData, const vrpn_TRACKERCB t);

// VRPN client init
int create_vrpn_tracker(void);

// TCP communication init
int create_server_socket(void);
int connect_with_client(void);

// VRPN server init
int create_vrpn_server(void);

//////////////////////////////////////////////////////////////////////////////
// MAIN
//////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
	cout << "VRPN client of Tracker " << probe_name.c_str() << endl;
	cout << "Recall: the Tracker must be on the scene and its position will be printed only if a change occurs in its pose" << endl;
	cout << "Press q to quit" << endl;
	
	// Create VRPN tracker to collect rigid bodies data
	create_vrpn_tracker();
	
	// Create TCP socket to send tracker data to client on defined port
	if (stream)
	{
		create_server_socket();
		connect_with_client();
	}

	// Create VRPN analog server to stream to procap
	//create_vrpn_server();

	// Main loop
	char c;
	while (1)
	{
		if (_kbhit())
		{
			if (_getch() == 'q')
			{
				break;
			}
		}

		// Listen to data from Motive
		vrpnTracker->mainloop();

		// Send analog data to VRPN manager
		//serverAnalog->mainloop();
		
		// Refresh manager
		//myConnection->mainloop();

		Sleep(10);
	}

	// Tell computer that we finished using sockets
	closesocket(server);
	WSACleanup();

	return 0;
}

//////////////////////////////////////////////////////////////////////////////
// VRPN CLIENT
//////////////////////////////////////////////////////////////////////////////

void VRPN_CALLBACK handle_tracker(void* userData, const vrpn_TRACKERCB t)
{
	/*// Print tracker content 
	cout << endl << probe_name.c_str() << " " << t.msg_time.tv_sec << "." << t.msg_time.tv_usec << endl;
	cout << "Tracker position '" << t.sensor << "' : " << t.pos[0] << "," << t.pos[1] << "," << t.pos[2] << endl;
	cout << "Tracker orientation " << t.sensor << "' : " << t.quat[0] << "," << t.quat[1] << "," << t.quat[2] << "," << t.quat[3] << endl;*/

	// Position (x, y, z) in [m]
	probe_pos[0] = t.pos[0];
	probe_pos[1] = t.pos[1];
	probe_pos[2] = t.pos[2];
	
	// Orientation (quaternion with x, y, z, w)
	probe_quat[0] = t.quat[0];
	probe_quat[1] = t.quat[1];
	probe_quat[2] = t.quat[2];
	probe_quat[3] = t.quat[3];

	// Send VRPN data to TCP client
	char message[256];

	snprintf(message, sizeof(message), "%s: %.4f %.4f %.4f %.4f %.4f %.4f %.4f",
		probe_name.c_str(), probe_pos[0], probe_pos[1], probe_pos[2],
		probe_quat[0], probe_quat[1], probe_quat[2], probe_quat[3]);

	cout << message << endl;

	if (stream)
	{
		// Send in TCP to client
		if (send(client, message, strlen(message), 0) == SOCKET_ERROR)
		{
			client_connected = false;
			cout << "Connection lost with client" << endl;
		}
	}
}

int create_vrpn_tracker(void)
{
	// Tracker URL (label@ip)
	string url = probe_name + '@' + vrpn_server_ip;

	// Get VRPN data with callback handle_tracker
	vrpnTracker = new vrpn_Tracker_Remote(url.c_str());
	vrpnTracker->register_change_handler(0, handle_tracker);

	return 0;
}

//////////////////////////////////////////////////////////////////////////////
// TCP COMMUNICATION
//////////////////////////////////////////////////////////////////////////////

int create_server_socket(void)
{
	// Tell computer that we use sockets
	if (WSAStartup(MAKEWORD(2, 2), &WSAData) != 0)
	{
		cout << "Error startup initialization: " << WSAGetLastError();
		return -1;
	}

	// Create server socket
	server = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	if (server == INVALID_SOCKET)
	{
		cout << "Error socket initialization: " << WSAGetLastError();
		return -1;
	}

	// Socket info
	addr.sin_addr.s_addr = htonl(INADDR_ANY); // No binding (server)
	addr.sin_family = AF_INET; // Internet
	addr.sin_port = htons(port); // Port

								 // Bind socket with address
	int res = bind(server, (SOCKADDR *)&addr, sizeof(addr));

	if (res != 0)
	{
		cout << "Error bind: " << WSAGetLastError();
		return -1;
	}

	// Define queue length
	res = listen(server, SOMAXCONN);

	if (res != 0)
	{
		cout << "Error listen: " << WSAGetLastError();
		return -1;
	}

	// Server started successfully
	cout << "Server started on port " << port << endl;

	return 0;
}

int connect_with_client(void)
{
	// Init client parameters to 0, will be filled with client parameters at connection
	from = { 0 };

	// Init adress length, will be filled with client parameters at connection
	int len = sizeof(addr);

	// Connect to client (blocking)
	client = accept(server, (SOCKADDR *)&from, &len);

	if (client == INVALID_SOCKET)
	{
		cout << "Error accept: ";
		return -1;
	}

	client_connected = true;
	cout << "Client connected from " << inet_ntoa(from.sin_addr) << endl;

	return 0;
}

//////////////////////////////////////////////////////////////////////////////
// VRPN SERVER
//////////////////////////////////////////////////////////////////////////////

int create_vrpn_server(void)
{
	// Connection to VRPN
	myConnection = new vrpn_Connection_IP();

	// Create analog instance
	serverAnalog = new myAnalog(myConnection);

	return 0;
}

myAnalog::myAnalog(vrpn_Connection *c) : vrpn_Analog("Analog0", c)
{
	vrpn_Analog::num_channel = 10;

	for (vrpn_uint32 i = 0; i < (vrpn_uint32)vrpn_Analog::num_channel; i++) {
		vrpn_Analog::channel[i] = vrpn_Analog::last[i] = 0;
	}
}

void myAnalog::mainloop(void)
{
	vrpn_gettimeofday(&_timestamp, NULL);
	vrpn_Analog::timestamp = _timestamp;

	// forcing values to change otherwise vrpn doesn't report the changes
	static float f = 0;
	f += 0.001f;

	for (int i = 0; i<vrpn_Analog::num_channel; i++)
	{
		// XXX Set your values here !
		channel[i] = i / 10.f + f;
	}

	// Send any changes out over the connection.
	vrpn_Analog::report_changes();

	server_mainloop();
}