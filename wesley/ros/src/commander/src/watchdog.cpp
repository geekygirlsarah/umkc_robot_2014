#include "watchdog.h"
#include <sstream>
using std::stringstream;
void WatchDOg::execute_pkill(string name){
	stringstream ss;
	ss << "pkill " << name;
	FILE * f = popen(ss.str().c_str(),"r");
	pclose(f);
}
