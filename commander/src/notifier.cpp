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
		bool green0;
		bool green1;
		bool yellow0;
		bool yellow1;
		bool red0;
		bool red1;
		fin >> name >> green0>>green1>>yellow0 >> yellow1 >> red0 >> red1;
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

