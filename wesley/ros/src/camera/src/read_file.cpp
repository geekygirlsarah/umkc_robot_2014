#include <ros/ros.h>
#include <wesley/arm_point.h>

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <sys/stat.h>
#include <vector>

using namespace ros;

struct point {
	int x, y, z;
	point(int x = 0, int y = 0, int z = 0) :
		x(x), y(y), z(z) { };
};

int main(int argc, char* argv[]) {
	std::ifstream fin;
	struct stat verify;
	if (stat("./position_tool.lst", &verify) == 0) {
		fin.open("./position_tool.lst", std::ifstream::in);
	} else {
		std::cerr << "unable to locate 'position_tool.lst' - fatal; bailing." << std::endl;
		return(60);
	}
	
	int lines, w;
	wesley::arm_point position[2][3][13];

//	while (!fin.eof()) {
		std::string buffer;
		int process = 0;	// FIND_TOOL
		for (int area = 0; area < 3; area++) {
			for(int pos = 0; pos < 13; pos++) {
				getline(fin, buffer);
				if (buffer[0] == '#' || buffer.size() == 0) {
					continue;
//					pos--;
				}
				std::stringstream ss;
			//	std::cout << "buffer: " << buffer << std::endl;
				ss << buffer;
				float x, y, z, p, r;
			//	std::cout << ss.str() << std::endl;
				ss >> x >> y >> z >> p >> r;
//				std::cout << x << ","
//						  << y << ","
//						  << z << ","
//						  << p << ","
//						  << r << std::endl;
				position[process][area][pos].direct_mode = true;
				position[process][area][pos].x = x; 
				position[process][area][pos].y = y;
				position[process][area][pos].z = z;
				position[process][area][pos].p = p;
				position[process][area][pos].r = r;
				position[process][area][pos].cmd = "id_tool: first identify";
				std::cout << position[process][area][pos] << std::endl;
				ss.str("");
			}
		}

		process = 1;
		for (int area = 0; area < 3; area++) {
			for (int pos = 0; pos < 5; pos++) {
				getline(fin, buffer);
				if (buffer[0] == '#' || buffer.size() == 0) {
					continue;
//					pos--;
				}
				std::stringstream ss;
			//	std::cout << "buffer: " << buffer << std::endl;
				ss << buffer;
				float x, y, z, p, r;
			//	std::cout << ss.str() << std::endl;
				ss >> x >> y >> z >> p >> r;
//				std::cout << x << ","
//						  << y << ","
//						  << z << ","
//						  << p << ","
//						  << r << std::endl;
				position[process][area][pos].direct_mode = true;
				position[process][area][pos].x = x; 
				position[process][area][pos].y = y;
				position[process][area][pos].z = z;
				position[process][area][pos].p = p;
				position[process][area][pos].r = r;
				position[process][area][pos].cmd = "id_tool: second confirm";
				std::cout << position[process][area][pos] << std::endl;
				ss.str("");
			}
		}

//	}

	fin.close();
	return(0);
}

