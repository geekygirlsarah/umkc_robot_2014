#include "notifier.h"

class ExitHandler{
private:
	LedNotifier ledNotifier;
	int flame, tool;
public:
	ExitHandler();

	void id_flame(int);
	void id_tool(int);
}
