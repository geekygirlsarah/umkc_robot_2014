#include "waitforgo.h"
#include "notifier.h"

int main(){
	try{
	LedNotifier ledNotifier(true);

	

	}
	catch(...){
		LedNotifier::throwGeneralFailure();
	}
	
	// main process list start here
	// 1) id_flame
	// 2) id_tool
	// 3) -------
	// 4) -------
}
