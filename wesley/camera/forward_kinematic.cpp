#include <fstream>
#include <iostream>
#include <string>
#include <cmath>
#include <sstream>

#define radians(deg)		((deg * 3.1415927) / 180)

using namespace std;

struct point {
	int x, y, z;
	point(int x = 0, int y = 0, int z = 0) :
		x(x), y(y), z(z) { };
};

struct point getxyz(int BASE, int SHOULDER, int ELBOW, int WRIST_P);

int main(int argc, char* argv[]) {
	if (argc != 2) {
		cerr << "you must include a file name of coordinates to process" << endl;
		return(20);
	}

	cout << "opening: " << argv[1] << endl;

	ifstream fin(argv[1]);

	string buffer;
	while(!fin.eof()) {
		getline(fin, buffer);
		stringstream ss;
		ss << buffer;
		string check;
		ss >> check;
		if (check == "actual") {
	//		cout << ss.str() << endl;
			char trash;
			int x, y, z, w;
			ss >> trash >> x >> y >> z >> w;
	//		cout << x << " " << y << " " << z << " " << w << endl;
			point pos = getxyz(x, y, z, w);
			cout << pos.x << " " << pos.y << " " << pos.z << " " << w << endl;
	//		cout << "read: " << buffer << endl;
		}
	}

	fin.close();

	
	return(0);
}

#define BASE_HGT	80.5
#define HUMERUS		146.5
#define ULNA		216.0
#define GRIPPER		96.5

struct point getxyz(int BASE, int SHOULDER, int ELBOW, int WRIST_P) {
	// this function should return the XYZ position of the arm.
	// do we worry about the roll of the wrist? shouldn't have to.
	//    the point of consideration lies along the wrists roll axis.
	double  cosB,  sinB;
	double hcosS, hsinS, sumofpiecesX;
	double ucosE, usinE, sumofpiecesY;
	double gcosW, gsinW, sumofpiecesZ;

	//	int B = arm[BASE].read();					double B_r = radians(B);
	//	int S = arm[SHOULDER].read();				double S_r = radians(S);
	//	int E = -(169 - S - arm[ELBOW].read());		double E_r = radians(E);
	//	int W = -(90 - arm[WRIST_P].read() - E);	double W_r = radians(W);
	double B_r = radians(BASE);
	double S_r = radians(SHOULDER);
	double E_r = radians(ELBOW);
	double W_r = radians(WRIST_P);

	 cosB = cos(B_r);
	 sinB = sin(B_r);

	// x and y components of various bones
	hcosS = HUMERUS * cos(S_r);
	ucosE = ULNA * cos(E_r);
	gcosW = GRIPPER * cos(W_r);

	// z components
	hsinS = HUMERUS * sin(S_r);
	usinE = ULNA * sin(E_r);
	gsinW = GRIPPER * sin(W_r);


	sumofpiecesX = cosB * (hcosS + ucosE + gcosW);
	sumofpiecesY = sinB * (hcosS + ucosE + gcosW);
	sumofpiecesZ = BASE_HGT + hsinS + usinE + gsinW;

	struct point pillow(sumofpiecesX, sumofpiecesY, sumofpiecesZ);

	return(pillow);
}

