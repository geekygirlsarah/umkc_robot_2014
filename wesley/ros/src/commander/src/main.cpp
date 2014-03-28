#include "exit_handlers.h"
#include <string>
using std::string;


/**
 * Executes a binary file at path and returns the exit code.
 *
 * The prefix is the begining to the path where the binary is located
 *
 * The mode can be left alone usually, it's set to write
 *
 * @return The integer exit code from the binary
 */
int executeBinary(string path,string prefix="./bin/", string mode="r");
/**
 * The main function. Contains an instance of the LedNotifier object
 *
 * All binaries should be handled in this function. 
 *
 * Fucntions should be written to handle the exit codes from the binarys
 */
int main(){
	Logger * logger = new Logger();
	ExitHandler exithandler(logger);

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
	logger->logStatus("Executing button_wait");
	exithandler.button_wait(executeBinary("button_wait"));
	logger->logStatus("Executing ID flame");
	exithandler.id_flame(executeBinary("id_flame"));
	logger->logStatus("Executing ID tool");
	exithandler.id_tool(executeBinary("id_tool"));
}

int executeBinary(string binaryName, string prefix, string mode ){
	string name = prefix + binaryName;
	FILE * f = popen(name.c_str(),mode.c_str());
	
	//Eror while opening file stream
	if(f == 0){
		return -2;
	}
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
