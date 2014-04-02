#include "watchdog.h"
#include <unistd.h>
#include <stdio.h>
#include <sstream>
#include <thread>
#include <chrono>
using std::stringstream;

void WatchDog::execute_pkill(string name){
	stringstream ss;
	ss << "pkill " << name;
	FILE * f = popen(ss.str().c_str(),"r");
	pclose(f);
}

void WatchDog::tick(){
	amountCalled++;
}

void WatchDog::operator()(string binaryName){
	amountCalled = 0;
	while(amountCalled < checkAmount){
		tick();
		std::this_thread::sleep_for(std::chrono::seconds(seconds_to_sleep));
	}
	execute_pkill(binaryName);
	/**Need to delete the memory allocated for this because it's a suicide thread*/
	delete this;
}

