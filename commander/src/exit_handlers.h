#include "notifier.h"

class ExitHandler{
private:
	LedNotifier ledNotifier;
	Logger* logger;
	int flame, tool;
public:
	ExitHandler(Logger* logger_);

	void id_flame(int);
	void id_tool(int);
};
