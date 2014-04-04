#include <ros/ros.h>
#include "runner.h"
#include <string>
#include <unistd.h>
#include <signal.h>
#include "armcommands.h"
using std::string;
/**
 * The main function. Contains an instance of the LedNotifier object
 *
 * All binaries should be handled in this function. 
 *
 * Fucntions should be written to handle the exit codes from the binarys
 *	// here -- the recommended manner in calling the binaries is as follows:
	//
	//    rosrun package binary arguments
	//
	// more research should be done to determine if we need an argument vector
	//    to pass through popen, or if it can handle long strings and execute
	//    those. rosrun will return the error code passed by the called binary.

	// main process list start here
	// 0) button_wait
	// 1) id_flame
	// 2) move_to_tools
	// 3) id_tool
	// 4) move_through_waves
	// 5) move_to_rig
	// 6) align_on_rig
	// 7) 

 */
int main(int argc, char* argv[]) {
	ros::init(argc, argv, "cmdr");
	ROS_INFO("CMDR :: main --> commander initializing");
	string parse_file = "/home/umkc/wesley/config";
	if (argc == 2) {			// someone passed us a leading path to
		parse_file = argv[1];	//    notify_id.txt
	}

	parse_file += "/notify_id.txt";
	Runner runner(parse_file);
	ROS_INFO("CMDR :: main --> logger and handler set up.");

//	ROS_INFO("CMDR :: main --> closing hand to avoid collision.");
//	grasp();

	ROS_WARN("CMDR :: main --> launching: button_wait");
	runner.button_wait();		
	// sleep for a couple of seconds to allow clearance of hands and feet.
	sleep(2);

	runner.id_flame();	
	

//	logger->logStatus("init -- opening hand.");
//	release();

	runner.cab_man(0,0);
//	logger->logStatus("Executing ID tool");
	ArmCommands::carry();

	runner.cab_man(0,1);

	return 0;

}
