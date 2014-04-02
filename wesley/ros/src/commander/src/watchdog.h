#include <string>
#include <unistd.h>
#include <signal.h>
#include <thread>
#include <map>
#include <stdio.h>
using std::map;
using std::string;

#ifndef WATCHDOG_H
#define WATCHDOG_H
/**
 * Watch dog is a functor object that should be called as such, don't worry it suicides itself and deallocates it's own memory
 * WatchDog* watch = new WatchDog; 
 * std::thread(watch(binaryName));
 */
class WatchDog{
	string name;
	int amountCalled;
	int checkAmount; /*<The time in near-seconds for the thread to be alive before it's killed */
	const int seconds_to_sleep = 1;
	//Tick through each process updating the amount of times it's been called in call_map and kills it if it's been checked checkAmount times
	void tick();
	void execute_pkill(string name);	
	 
public:
	WatchDog(){checkAmount=1000;}
	WatchDog(int amountOfTicksBeforeKilled){checkAmount=amountOfTicksBeforeKilled;}
	void operator()(string binaryName);

};
#endif

