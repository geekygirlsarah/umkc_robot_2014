#include "waitforgo.h"
#include "notifier.h"
#include <string>
using std::string;
/**
 * Executes a binary file at path and returns the exit code.
 *
 * The prefix is the begining to the path where the binary is located
 *
 * The mode can be left alone usually, it's set to write
 *
 * @return The integer exit code from the binary
 */
int executeBinary(string path,string prefix="./bin/", string mode="r");
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
	executeBinary("id_flame");
	executeBinary("id_tool");
}

int executeBinary(string binaryName, string prefix, string mode ){
	FILE * f = popen(prefix + binaryName,mode);
	
	//Eror while opening file stream
	if(f == 0){
		return -2;
	}
	//Get return value, don't ask why it's this but it is. It's from the stack overflow on popen. 
	return pclose(f)/256;
