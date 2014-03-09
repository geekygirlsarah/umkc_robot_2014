#include "waitforgo.h"
#include "exceptor.h"

int main(){
	try{
	Exceptor exceptor(true);




	}
	catch(...){
		Exceptor::throwGeneralFailure();
	}

}
