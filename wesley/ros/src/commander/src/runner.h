#include "notifier.h"
#include "unistd.h"
#include <sstream>
class Runner{
private:
	ros::NodeHandle handle;
	LedNotifier ledNotifier;
	int flame, shape;
	/**
	 * Puts the robot in an infinite loop if it has to give up	
	 */
	static void die(){
		ROS_INFO("Unrecoverable error, going into death protocol");
		ArmCommands::giveup();
		ledNotifier.lightLeds("death",true);
		ROS_INFO("Spinning forever as a result of death");
		while(true){sleep(1);}
	}
	/*
	 * Logs a return code to ros info
	 */
	void logReturnCode(string binary, int code){
		ROS_INFO(binary + "returned value (%d)", code);	
	}
	/**
	 * Used to log a status from a binary return code
	 */	
	void logStatus(string context, string name, bool throwMatchingLedCode = true){
		ROS_INFO(context + " logged a status of  " + name);
		if(throwMatchingLedCode){
			ledNotifier.throwLedCode(name);
		}
	}
	/**
	 * Called when an unspecified failure has been found
	 */
	void generalFailure(string context, bool giveup = false){
		ledNotifier.throwGeneralFailure();
		ROS_INFO("General Failure in " + context);
		if(giveup){die();}
	}

public:
/**
 * Executes a binary file at path and returns the exit code.
 *
 * The prefix is the begining to the path where the binary is located
 *
 * The mode can be left alone usually, it's set to read 
 *
 * @return The integer exit code from the binary
 */
	static int executeBinary(string path,string prefix="", string mode="r");
	/**
	 * Construcs an exithandler class
	 *
	 * This class should be used to handle the return codes for wesleys binaries
	 */
	Runner(string parse_file);
	/**
	 * Handle for button_wait
	 */
	void button_wait();
	/**
	 * Handles the ID flame codes
	 * 0 - throws id_flame_notfound led and sets flame to 0
	 * 1 - sets flame to 1 and throws id_flame_square
	 * 2 - sets flame to 2 and throws id_flame_triangle
	 * 3 - sets flame to 3 and throws id_flame_circle 
	 * otherwise - throws cant_find_file
	 */
	void id_flame();

	/**
	 * Handles the id_tool binary
	 *
	 */
	void id_tool();

	void cab_man(int id_1,int id_2);
};
