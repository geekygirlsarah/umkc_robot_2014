
/*
practice talking with ros and custom msgs
*/
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;


std_msgs::String str_msg;
ros::Publisher chatter("arduinoToBoard", &str_msg);
void packet_catch(const std_msgs::String& packet);  
ros::Subscriber<std_msgs::String> listener("boardToArduino", &packet_catch);


//ros msg catching time!
void packet_catch(const std_msgs::String& packet)  {
    
}

char hello[13] = "hello world!";


char debug[]= "debug statements";
char info[] = "infos";
char warn[] = "warnings";
char error[] = "errors";
char fatal[] = "fatalities";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(listener);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  
  nh.logdebug(debug);
  nh.loginfo(info);
  nh.logwarn(warn);
  nh.logerror(error);
  nh.logfatal(fatal);
  
  nh.spinOnce();
  delay(500);
}

