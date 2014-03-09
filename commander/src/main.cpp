#include "waitforgo.h"
#include "notifier.h"

int main(){
	try{
	LedNotifier ledNotifier(true);



	}
	catch(...){
		LedNotifier::throwGeneralFailure();
	}

}
