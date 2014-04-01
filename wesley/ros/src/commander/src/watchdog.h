#include <string>
#include <unistd.h>
#include <signal.h>
using std::string;

class WatchDog{
	int timesCalled;
	void callback(string binaryName)
	void execute_pkill(string name);	
public:
	WatchDog(){timesCalled = 0;}
	WatchDog(int times){timesCalled=times;};
	void operator()(string binaryName){
		signal(SIGALARM,callback(binaryName);

