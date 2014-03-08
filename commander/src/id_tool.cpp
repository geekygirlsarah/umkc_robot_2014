#include "id_tool.h"
#include <unistd.h>
#include <sys/types.h> /* for pid_t */
#include <sys/wait.h> /* for wait */
using namespace std;

int id_tool::execute(){
	 /*Spawn a child to run the program.*/
    pid_t pid=fork();
    if (pid==0) { /* child process */
        static char *argv[]={"echo","Foo is my name.",NULL};
        exec("/bin/id_tool");
        exit(127); /* only if execv fails */
    }
    else { /* pid!=0; parent process */
        waitpid(pid,0,0); /* wait for child to exit */
    }
    return 0;
}

