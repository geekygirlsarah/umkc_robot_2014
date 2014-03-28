#include "exit_handlers.h"
ExitHandler::ExitHandler(Logger* logger_){
	// this initializer has changed somewhat - the default parse() does not
	//    point to a valid file at the moment.
//	LedNotifier ledNotifier(true);
//	LedNotifier ledNotifier;
	// this is the currently correct location of the file.
	ledNotifier.parse("/home/umkc/wesley/umkc_robot_2014_arduino/wesley/config/notify_id.txt");
	logger = logger_;
}

//button_wait will call button_wait and wait for it to unblock.
//currently, it only returns one error code: 99 on can't find file
void ExitHandler::button_wait(int returnCode) {
	switch(returnCode) {
		case 99:
			ledNotifier.throwLedCode("cant_find_file");
			break;
		case 0:
			ledNotifier.throwLedCode("ready_set_go");
			break;
		default:
			ledNotifier.throwLedCode("general_failure");
			break;
	}
}
//id_flame will take our return code from id_flame and throw led 
//notification based on the outcome
void ExitHandler::id_flame(int returnCode){
	//switch based on return code 
	flame = 0;
	switch(returnCode)
	{
	case 0:
	
		ledNotifier.throwLedCode("id_flame_notfound");
		break;
	
	case 1:
	
		flame = 1; 
		ledNotifier.throwLedCode("id_flame_square");
		break;
	
	case 2:
	
		flame = 2;
		ledNotifier.throwLedCode("id_flame_triangle");
		break;
	
	case 3:
	
		flame = 3;
		ledNotifier.throwLedCode("id_flame_circle");
		break;
	
	case 10:
	case 20:
	case 30:
	case 40:
	
		ledNotifier.throwLedCode("cant_find_file");
		break;
	
	case -1:
		ledNotifier.throwLedCode("general_failure");
		break;
	}
}
//id_tool function will take our return code from id_tool program and 
//throw led notification based on the outcome
void ExitHandler::id_tool(int returnCode)
{
	switch(returnCode){
		case 0:
			ledNotifier.throwLedCode("id_tool_failure");
			break;
		case -1:
			ledNotifier.throwLedCode("general_failure");
			break;
		default:	
			//set our private variable tool = to returnCode for documentation
			tool = returnCode;
			break;
	}
}

