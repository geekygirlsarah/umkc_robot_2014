#include "id_tool.h"
#include <stdio.h>


/**
 * 1 = No error, tool found
 * -1 = General error
 */
int id_tool::execute(){
	FILE * f = popen("./bin/id_tool","r");
	//Eror while opening file stream
	if(f == 0){
		return -1;
	}
	//Get return value, don't ask why it's this but it is.
	return pclose(f)/256;


}

