#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

using namespace std;

struct point {
	int x, y, z;
	point(int x = 0, int y = 0, int z = 0) :
		x(x), y(y), z(z) { };
};

int main(int argc, char* argv[]) {
	ifstream fin("coordinates_tools.lst");

	int lines, w;
	point position[2][3][5];
	while (!fin.eof()) {
		string buffer;
		getline(fin, buffer);
		if (buffer[0] == '#' || buffer.size() == 0) {
			continue;
		}
		stringstream ss;
		ss << buffer;
		cout << ss.str() << endl;

	//	fin >> lines;
	//	cout << lines << endl;
	//	while (lines-- > 0) {
	//		fin >> position.x
	//			>> position.y
	//			>> position.z
	//			>> w;
	//		cout << "(" << position.x << ", "
	//			<< position.y << ", "
	//			<< position.z << ", "
	//			<< w << ")" << endl;
	//	}
	}

	fin.close();
	return(0);
}

