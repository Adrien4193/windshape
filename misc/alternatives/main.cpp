//
// Simple example to demonstrate how to use DroneCore.
//
// Author: Julian Oes <julian@oes.ch>

#include <chrono>
#include <cstdint>
#include <dronecore/action.h>
#pragma comment(lib, "dronecore_action.lib")
#include <dronecore/dronecore.h>
#pragma comment(lib, "dronecore.lib")
#include <dronecore/telemetry.h>
#pragma comment(lib, "dronecore_telemetry.lib")
#include <iostream>
#include <thread>

using namespace dronecore;
using namespace std::this_thread;
using namespace std::chrono;


int main(int argc, char **argv)
{
	DroneCore dc;
	std::string connection_url;
	ConnectionResult connection_result;

	bool discovered_system = false;

	connection_url ="tcp://192.168.8.1:5760";
	connection_result = dc.add_any_connection(connection_url);

	if (connection_result != ConnectionResult::SUCCESS) {
		std::cout << "Connection failed: "
			<< connection_result_str(connection_result)
			<< std::endl;
		return 1;
	}

	std::cout << "Waiting to discover system..." << std::endl;
	dc.register_on_discover([&discovered_system](uint64_t uuid) {
		std::cout << "Discovered system with UUID: " << uuid << std::endl;
		discovered_system = true;
	});

	// We usually receive heartbeats at 1Hz, therefore we should find a system after around 2 seconds.
	sleep_for(seconds(2));

	if (!discovered_system) {
		std::cout << "No system found, exiting." << std::endl;
		return 1;
	}

	// We don't need to specify the UUID if it's only one system anyway.
	// If there were multiple, we could specify it with:
	// dc.system(uint64_t uuid);
	System &system = dc.system();

	auto telemetry = std::make_shared<Telemetry>(system);
	auto action = std::make_shared<Action>(system);

	// We want to listen to the altitude of the drone at 1 Hz.
	const Telemetry::Result set_rate_result = telemetry->set_rate_position(1.0);
	if (set_rate_result != Telemetry::Result::SUCCESS) {
		std::cout <<  "Setting rate failed:" << Telemetry::result_str(
			set_rate_result) << std::endl;
		return 1;
	}


	// Set up callback to monitor altitude while the vehicle is in flight
	telemetry->position_async([](Telemetry::Position position) {
		std::cout
			<< "Altitude: " << position.relative_altitude_m << " m"
			<< std::endl;
	});

	// Check if vehicle is ready to arm
	while (telemetry->health_all_ok() != true) {
		std::cout << "Vehicle is getting ready to arm" << std::endl;
		sleep_for(seconds(1));
	}

	// Arm vehicle
	std::cout << "Arming..." << std::endl;
	const ActionResult arm_result = action->arm();

	if (arm_result != ActionResult::SUCCESS) {
		std::cout << "Arming failed:" << action_result_str(
			arm_result) << std::endl;
		return 1;
	}
	/*
	// Take off
	std::cout << "Taking off..." << std::endl;
	const ActionResult takeoff_result = action->takeoff();
	if (takeoff_result != ActionResult::SUCCESS) {
		std::cout << "Takeoff failed:" << action_result_str(
			takeoff_result) << std::endl;
		return 1;
	}

	// Let it hover for a bit before landing again.
	sleep_for(seconds(10));

	std::cout << "Landing..." << std::endl;
	const ActionResult land_result = action->land();
	if (land_result != ActionResult::SUCCESS) {
		std::cout << "Land failed:" << action_result_str(
			land_result) << std::endl;
		return 1;
	}
	*/
	// We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
	sleep_for(seconds(5));
	std::cout << "Finished..." << std::endl;
	return 0;
}