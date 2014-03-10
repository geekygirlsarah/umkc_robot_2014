#include "notifier.h"
#include <iostream>
#include <fstream>
#include <string>
using std::string;
using std::ifstream;

void LedNotifier::parse(const char* parseFileName){
	ifstream fin(parseFileName);
	if(!fin){
		throwGeneralFailure();
	}
	while(fin.good()){
		string name;
		int green0in;
		int green1in;
		int yellow0in;
		int yellow1in;
		int red0in;
		int red1in;
		fin >> name >> green0in>>green1in>>yellow0in >> yellow1in >> red0in >> red1in;
		//Stupid way to convert these two booleans but yolo
		bool green0 = (green0in == 1);
		bool green1 = (green1in == 1);
		bool yellow0 = (yellow0 == 1);
		bool yellow1 = (yellow1 == 1);
		bool red0 = (red0in == 1);
		bool red1 = (red1in == 1);
		notificationMap[name] = LedArray(green0,green1,yellow0,yellow1,red0,red1);
	}
}	

bool LedNotifier::throwLedCode(string code, bool throwGeneralErrorOnFailure){
	if(notificationMap.find(code) == notificationMap.end()){
		if(throwGeneralErrorOnFailure)
			throwGeneralFailure();
		return false;
	}
	LedArray ledArray = notificationMap[code];
	lightLeds(ledArray.green0,ledArray.green1,ledArray.yellow0,ledArray.yellow1,ledArray.red0,ledArray.red1);
	return true;
}

void LedNotifier::lightLeds(bool green0, bool green1, bool yellow0, bool yellow1, bool red0, bool red1){
	return; //todo implement this
}
LedNotifier::LedNotifier(bool parseOnConstruction){
	if(parseOnConstruction)
		parse();
}

