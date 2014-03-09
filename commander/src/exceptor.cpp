#include "exceptor.h"
#include <iostream>
#include <fstream>

using std::ifstream;

void Exceptor::parse(){
	ifstream fin("error_id.txt");
	if(!fin){
		throwGeneralFailure();
	}
	while(fin.good()){
		string error_name;
		bool green0;
		bool green1;
		bool yellow0;
		bool yellow1;
		bool red0;
		bool red1;
		fin >> error_name >> green0>>green1>>yellow0 >> yellow1 >> red0 >> red1;
		errorMap[error_name] = LedArray(green0,green1,yellow0,yellow1,red0,red1);
	}
}	

bool Exceptor::throwLedCode(string code, bool throwGeneralErrorOnFailure){
	if(errorMap.find(code) == errorMap.end()){
		if(throwGeneralErrorOnFailure)
			throwGeneralFailure();
		return false;
	}
	LedArray ledArray = errorMap[code];
	lightLeds(ledArray.green0,ledArray.green1,ledArray.yellow0,ledArray.yellow1,ledArray.red0,ledArray.red1);
	return true;
}

void Exceptor::lightLeds(bool green0, bool green1, bool yellow0, bool yellow1, bool red0, bool red1){
	return; //todo implement this
}
Exceptor::Exceptor(bool parseOnConstruction){
	if(parseOnConstruction)
		parse();
}

