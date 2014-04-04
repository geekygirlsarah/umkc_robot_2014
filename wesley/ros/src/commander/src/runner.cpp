#include "runner.h" 
Runner::Runner(string parse_file) {
	ROS_INFO("EXIT :: (log, nh, str) --> entering.");
	ledNotifier.init_handle(handle);
	ROS_INFO("EXIT :: (log, nh, str) --> notifier created.");
	ledNotifier.parse(parse_file.c_str());
	ROS_INFO("EXIT :: (log, nh, str) --> notifier parsed file..");
	ROS_INFO("EXIT :: (log, nh, str) --> leaving.");
}

//button_wait will call button_wait and wait for it to unblock.
//currently, it only returns one error code: 99 on can't find file
void Runner::button_wait() {
	int returnCode = executeBinary("rosrun commander button_wait");
	switch(returnCode) {
		case 99:
			logStatus("button_wait","cant_find_file");
			break;
		case 0:
			ledNotifier.throwLedCode("ready_set_go");
			logStatus("button_wait","read_set_go");
			break;
		default:
			generalFailure("button_wait");
	}
}
//id_flame will take our return code from id_flame and throw led 
//notification based on the outcome
void Runner::id_flame(){
	int returnCode = executeBinary("rosrun camera id_flame");
	string name = "id_flame";
	logReturnCode(name,returnCode);	
	//switch based on return code 
	shape = 0;
	switch(returnCode)
	{
	case 0:
	
		logStatus(name,"id_flame_notfound");
		break;
	
	case 1:
	
		shape = 1; 
		logStatus(name,"id_flame_square");
		break;
	
	case 2:
	
		shape = 2;
		logStatus(name,"id_flame_triangle");
		break;
	
	case 3:
	
		shape = 3;
		logStatus(name,"id_flame_circle");
		break;
	
	case 10:
	case 20:
	case 30:
	case 40:
	
		logStatus(name,"cant_find_file");
		die();
		break;
	
	case 60:
		ROS_ERROR("CMDR :: id_flame --> return 60! indicates no fire found. no point going on; bailing.");
		die();
		break;
	case 50:
		ROS_ERROR("CMDR :: id_flame --> return 50! cannot open camera. fatal, but recoverable. bailing.");
		die();
		break;
	default:
		generalFailure("id_flame");
		break;
	}
	//The things they carried
	//Parks in carry spot
	ArmCommands::carry();
}
//id_tool function will take our return code from id_tool program and 
//throw led notification based on the outcome
void Runner::id_tool()
{
	std::stringstream ss;
	ss <<  "rosrun camera id_tool " << shape; 
	int returnCode = executeBinary(ss.str());
	logReturnCode(ss.str(),returnCode);
	switch(returnCode){
		case 0:
			logStatus("id_tool","id_tool_failure");
			break;
		case -1:
			generalFailure("id_tool");
			break;
		default:	
			//set our private variable tool = to returnCode for documentation
			tool = returnCode;
			std::stringstream ss;
			ss << "found tool number" << tool;
			logStatus("id_tool",ss.str());
			break;
	}
	ArmCommands::carry();
}
void Runner::cab_man(int id_1, int id_2){
	std::stringstream ss;
	ss << "rosrun commander cabman " << id_1 << " " << id_2;

	std::stringstream logstream;
	ss << "Cabman run with arguements" << id_1 << " " << id_2;
	string log_out = ss.str();
	ROS_INFO(log_out);

	int returnCode = executeBinary(ss.str());

	logReturnCode("cab_man",returnCode);
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

void Runner::die(){
		ROS_INFO("Unrecoverable error, going into death protocol");
		ArmCommands::giveup();
		ledNotifier.throwLedCode("death",true);
		ROS_INFO("Spinning forever as a result of death");
		while(true){sleep(1);}
}

void Runner::logReturnCode(string binary, int code){
	ROS_INFO(binary + "returned value (%d)", code);	
}

void Runner::logStatus(string context, string name, bool throwMatchingLedCode){
	ROS_INFO(context + " logged a status of  " + name);
	if(throwMatchingLedCode){
		ledNotifier.throwLedCode(name);
	}
}
void Runner::generalFailure(string context, bool giveup){
	ledNotifier.throwGeneralFailure();
	ROS_INFO("General Failure in " + context);
	if(giveup){die();}
}


void Runner::moveArmState(ArmState state){
	switch(state){
		case GRASP:
			ArmCommands::grasp();
			break;
		case RELEASE:
			ArmCommands::release();
			break;
		case PARK:
			ArmCommands::park();
			break;
		case CARRY:
			ArmCommands::carry();
			break;
		case GIVEUP:
			ArmCommands::giveup();
			break;
	}
}
