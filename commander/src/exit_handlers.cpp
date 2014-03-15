#include "exit_handlers.h"
ExitHandler::ExitHandler(){
	try{
		LedNotifier ledNotifier(true);
	}
	catch(...){
		LedNotifier::throwGeneralFailure();
	}
}

void id_flame(int returnCode){
	if(returnCode == -1){
	       ledNotifier.throwLedCode("id_flame_error");
	}
	else{
		flame = returnCode;
		ledNotifier.throwLedCode("id_flame_success");
	}
}
