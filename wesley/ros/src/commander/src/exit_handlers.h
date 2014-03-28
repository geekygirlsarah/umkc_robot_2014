#include "notifier.h"
#include "logger.h"
class ExitHandler{
private:
	LedNotifier ledNotifier;
	Logger* logger;
	int flame, tool;
public:
	/**
	 * Construcs an exithandler class
	 *
	 * This class should be used to handle the return codes for wesleys binaries
	 */
	ExitHandler(Logger* logger_);
	ExitHandler(Logger* logger_, ros::NodeHandle* handle, string parse_file);
	/**
	 * Handle for button_wait
	 */
	void button_wait(int);
	/**
	 * Handles the ID flame codes
	 * 0 - throws id_flame_notfound led and sets flame to 0
	 * 1 - sets flame to 1 and throws id_flame_square
	 * 2 - sets flame to 2 and throws id_flame_triangle
	 * 3 - sets flame to 3 and throws id_flame_circle 
	 * otherwise - throws cant_find_file
	 */
	void id_flame(int);

	/**
	 * Handles the id_tool binary
	 *
	 * 0 - throw id_tool_failure
	 * else - set tool to returnCode
	 */
	void id_tool(int);
};
