/**
Class for simply interfacing with the motor_cmd class using ros.

The message is defined as such
unit8 - mode
float32 - payload

mode is a char which represents which function to call

payload is the arg supplied to the motor_cmd function which is called.

if payload is -1, the overloaded run always command will be run. 

if the mode does not take a payload, the payload will be ignored.

modes are defined as follows:



*/


#include "../wesley/mega/libraries/sabertooth_mtrcmd/motor_cmd.h"
#include <ros.h>

motor_cmd Motor_Cmd; //< The object to call functions from

ros::NodeHandle nodeHandle;

Message::Message msg; //todo this needs to be actually implemented


ros::Publisher publisher("mega_pub", &msg);
void onMessageRecieve(Message:Message &msg); //<Prototype for callback function
ros::Subscriber<Message::Message> subscriber("mega_sub",&onMessageRecieve);

//Some variables so there aren't magic numbers
const int serialPort = 9600;

/**
Called at the begining, sets up required functions and initailizes ROS
*/
void setup(){
	Serial.begin(serialPort); //Initialzes Serial 	
	Motor_Cmd.begin(serialPort);//This might not be correct, need to check motor_cmd class
}
/**
initailizes all required ros functions and objects
*/
void initROS(){
	nodeHandle.initNode();
	nodeHandle.advertise(publisher);
	nh.subscribe(subscriber);
}
//Spins ros and echos the recieved command once it's completed
void loop(){
	nh.spinOnce();
	//todo echo recieved command here
}
//Handles the message once it's recieved based on spec at begining of file
void onMessageRecieve(Message::Message &msg){
	//Todo implement and make sure to update doc at top of file with defined modes and payloads
}
