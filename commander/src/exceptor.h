//This is the object for handling exceptions and transforming them into leds
#include <map>
#include <string>
using std::map;
using std::string;


#ifndef EXCEPTOR_H
#define EXCEPTOR_H
class Exceptor{
private:
	struct LedArray{
		bool green0;
		bool green1;
		bool yellow0;
		bool yellow1;
		bool red0;
		bool red1;
		LedArray() : green0(false),green1(false),yellow0(false),yellow1(false),red0(false),red1(false) {}
		LedArray(bool green0, bool green1, bool yellow0, bool yellow1, bool red0, bool red1) : green0(green0), green1(green1),yellow0(yellow0),yellow1(yellow1),red0(red0),red1(red1) {}
	};

	map<string,LedArray> errorMap;

	static void lightLeds(bool green0, bool green1, bool yellow0, bool yellow1, bool red0, bool red1);

public:
	Exceptor(bool parseOnConstruction=false);
	/**
	 * Parses the text file "error_id.txt"
	 *
	 * The format of this file goes 
	 * failure_id (Needs to be one word)
	 * green led0
	 * green led1
	 * yellow led0
	 * yellow led1
	 * red led0 
	 * red led1
	 * 
	 * in the event of a parse failure the leds will throw a general failure
	 */
	void parse();

	/**
	 * Throws a new led code based on previously parsed file. if throwGeneralErrorOnFailure is true then if no code is found the function will return false and light the general failure led
	 * @return true if code was found
	 *
	 */
	bool throwLedCode(string code, bool throwGeneralErrorOnFailure = false);
	
	/**
	 * A function that will light the general failure leds
	 * note: doesn't rely on a successful parse
	 */
	static void throwGeneralFailure(){lightLeds(false,false,false,false,true,true);}
};
#endif
