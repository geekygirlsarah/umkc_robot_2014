#include <ros/ros.h>
#include "exit_handlers.h"
#include <string>
#include <unistd.h>
#include <signal.h>
#include <watchdog.h>
//#include <thread.h> Include if using watchdog
using std::string;

#define grasp() executeBinary("rostopic pub -1 /arm/put/point wesley/arm_point '{direct_mode: false, cmd: grasp}'", "");
#define release() executeBinary("rostopic pub -1 /arm/put/point wesley/arm_point '{direct_mode: false, cmd: release}'", "");
#define park() executeBinary("rostopic pub -1 /arm/put/point wesley/arm_point '{direct_mode: false, cmd: park}'", "");
#define carry() executeBinary("rostopic pub -1 /arm/put/point wesley/arm_point '{direct_mode: false, cmd: carry}'", "");
#define giveup() executeBinary("rostopic pub -1 /arm/put/angle wesley/arm_angle 90 90 180 90 90 120", "");

/**
 * Executes a binary file at path and returns the exit code.
 *
 * The prefix is the begining to the path where the binary is located
 *
 * The mode can be left alone usually, it's set to write
 *
 * @return The integer exit code from the binary
 */
int executeBinary(string path,string prefix="/home/umkc/wesley/lib/", string mode="r");
/**
 * The main function. Contains an instance of the LedNotifier object
 *
 * All binaries should be handled in this function. 
 *
 * Fucntions should be written to handle the exit codes from the binarys
 */
int main(int argc, char* argv[]) {
	ros::init(argc, argv, "cmdr");
	ros::NodeHandle nh;
	ROS_INFO("CMDR :: main --> commander initializing");

//	Logger * logger = new Logger(&nh);
	string parse_file = "/home/umkc/wesley/config";
	if (argc == 2) {			// someone passed us a leading path to
		parse_file = argv[1];	//    notify_id.txt
	}

	parse_file += "/notify_id.txt";
	//	ExitHandler exithandler(logger, &nh, parse_file);
	ExitHandler exithandler(&nh, parse_file);
	ROS_INFO("CMDR :: main --> logger and handler set up.");

	// here -- the recommended manner in calling the binaries is as follows:
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
//	ROS_INFO("CMDR :: main --> closing hand to avoid collision.");
//	grasp();

//	logger->logStatus("Executing button_wait");
	ROS_WARN("CMDR :: main --> launching: button_wait");
	exithandler.button_wait(executeBinary("rosrun commander button_wait", ""));
	// sleep for a couple of seconds to allow clearance of hands and feet.
	sleep(2);

//	logger->logStatus("Executing ID flame");
	int tool = 0;
	exithandler.id_flame(tool = executeBinary("rosrun camera id_flame", ""));
	ROS_INFO("ID_FLAME returned value (%d)", tool);
	switch(tool) {
		case 60:
			ROS_ERROR("CMDR :: id_flame --> return 60! indicates no fire found. no point going on; bailing.");
			giveup();
			return(tool);
			break;
		case 50:
			ROS_ERROR("CMDR :: id_flame --> return 50! cannot open camera. fatal, but recoverable. bailing.");
			giveup();
			return(tool);
			break;
	}

//	logger->logStatus("init -- parking in carry spot.");
	carry();

//	logger->logStatus("init -- opening hand.");
//	release();

	executeBinary("rosrun commander cabman 0 0", "");

//	logger->logStatus("Executing ID tool");
	std::stringstream ss;
	ss <<  "rosrun camera id_tool " << tool  << " /home/umkc/wesley/config/position_tool.lst";
	exithandler.id_tool(executeBinary(ss.str(), ""));

	carry();


	executeBinary("rosrun commander cabman 0 1", "");


}

int executeBinary(string binaryName, string prefix, string mode ){
	string name = prefix + binaryName;
	FILE * f = popen(name.c_str(),mode.c_str());
	
	//Eror while opening file stream
	if(f == 0){
		return -2;
	}
	/**
	 * Uncomment this to turn on watchdogging 
	//WatchDog watch;
	//std::thread(watch,binaryName);
	*/
	//Get return value, don't ask why it's this but it is. It's from the stack overflow on popen. 
	//
	// http://bytes.com/topic/c/answers/131694-pclose-returning-termination-status-command#post472837
	//
	// As specified in the above link, pclose encodes the child process' exit code
	//    in the high 16bits of an integer. pclose return values are based on wait4,
	//    which is based on waitpid. reading those two man pages (man 2 wait4, or
	//    man 2 waitpid) did not shed any extra light on this.
	return pclose(f)/256;
}


