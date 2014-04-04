#ifndef RUNNER_H
#define RUNNER_H

#include "notifier.h"
#include "unistd.h"
#include <sstream>
#include <ros/ros.h>
#include <ros/console.h>
/**
 * Executes a binary file at path and returns the exit code.
 *
 * The prefix is the begining to the path where the binary is located
 *
 * The mode can be left alone usually, it's set to read 
 *
 * @return The integer exit code from the binary
 */
int executeBinary(string path,string prefix="", string mode="r");

class Runner{

private:
	LedNotifier ledNotifier;
	int flame, shape, tool;

	/**
	 * Puts the robot in an infinite loop if it has to give up	
	 */
	void die();
	/*
	 * Logs a return code to ros info
	 */
	void logReturnCode(string binary, int code);
			
	/**
	 * Used to log a status from a binary return code
	 */	
	void logStatus(string context, string name, bool throwMatchingLedCode = true);
	/**
	 * Called when an unspecified failure has been found
	 */
	void generalFailure(string context, bool giveup = false);


public:
	ros::NodeHandle* handle;

	enum ArmState {GRASP,RELEASE,PARK,CARRY,GIVEUP};
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
	/**
	 * Manages vickys program
	 */
	void cab_man(int id_1,int id_2);
	/**
	 * Used to directly call an arm state, should probabbly only be used in neccessray situations, most arm stuff should be handled in a specific binary run Call
	 */
	void moveArmState(ArmState state);
};

#ifndef ArmCommands
#define ArmCommands
namespace ArmCommands{
	int grasp(){return executeBinary("rostopic pub -1 /arm/put/point wesley/arm_point '{direct_mode: false, cmd: grasp}'", "");}
	int release() {return executeBinary("rostopic pub -1 /arm/put/point wesley/arm_point '{direct_mode: false, cmd: release}'", "");}
	int park() {return executeBinary("rostopic pub -1 /arm/put/point wesley/arm_point '{direct_mode: false, cmd: park}'", "");}
	int carry() {return executeBinary("rostopic pub -1 /arm/put/point wesley/arm_point '{direct_mode: false, cmd: carry}'", "");}
	int giveup() {return executeBinary("rostopic pub -1 /arm/put/angle wesley/arm_angle 90 90 180 90 90 120", "");}
};
#endif

#endif

