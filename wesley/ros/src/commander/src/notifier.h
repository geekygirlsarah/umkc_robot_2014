  /* Notifier.h
   * written by: Chris Denniston
   * date:  Sun Mar  9 15:12:14 CDT 2014
   *
   * PURPOSE: To allow ease of use when throwing Led status notifications
   *
   * 
   *
   */
#include <map>
#include <string>
#include <ros/ros.h>

using std::map;
using std::string;


#ifndef LedNotifier_H
#define LedNotifier_H
class LedNotifier{
private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	
	struct LedArray{
		bool green0;
		bool green1;
		bool yellow0;
		bool yellow1;
		bool red0;
		bool red1;
		LedArray() : green0(false),green1(false),yellow0(false),yellow1(false),red0(false),red1(false) {}
		LedArray(bool green0, bool green1,
				 bool yellow0, bool yellow1,
				 bool red0, bool red1) : 
			green0(green0),	green1(green1),
			yellow0(yellow0), yellow1(yellow1),
			red0(red0), red1(red1)
		{ };
	};

	map<string,LedArray> notificationMap;

	void lightLeds(bool green0, bool green1, bool yellow0, bool yellow1, bool red0, bool red1);

public:
	/**
	 * Constructor for LedNotifier.
	 *
	 * @param parseOnConstruction if set to true, will parse default file on construction
	 */
	LedNotifier(bool parseOnConstruction=false);
	/**
	 * Constructor for LedNotifier.
	 *
	 * @param parseOnConstruction if set to true, will parse default file on construction
	 *  - can be given any of set of 0 to 6 led status and those will be lit
	 *    on initial creation of the class.
	 */
	LedNotifier(bool grn1, bool grn2,
				bool ylw1, bool ylw2,
				bool red1, bool red2,
				bool parseOnConstruction=false);
		
	/**
	 * Parses a text file
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
	 *
	 * multiple files can be parsed. If there are overlaps than the most recently parsed code will stay.
	 *
	 * @param parseFileName The text file to parse
	 * returns true on success, false on failure.
	 */
	bool parse(const char* parseFileName="./res/notify_id.txt");

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
	void throwGeneralFailure(){lightLeds(false,false,false,true,true,true);}

	/**
	 * returns a list of the known status codes as parsed from the file.
	 *  does not work..?
	 */
	string cat_codes();
};
#endif
