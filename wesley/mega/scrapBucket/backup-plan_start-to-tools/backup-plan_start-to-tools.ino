

#include <Distance2D120X.h>

#include <DistSmoother.h>

#include <motor_cmd.h>

#include <QuadEncoder.h>

#include <WallFollower2.h>


#include <ros.h>
#include <std_msgs/Empty.h>




motor_cmd Saber;


//
QuadEncoder encoder;
WallFollow follower;
DistSmoother IR_R;

//



bool go = true;
bool backup = true;

ros::NodeHandle nh;
std_msgs::Empty motor_state;
ros::Publisher pub("/motor/chase/res", &motor_state);

bool waiting = true;
void block_wait_drive(const std_msgs::Empty& msg) {
  // whatever..
  waiting = false;
}
ros::Subscriber<std_msgs::Empty> sub("/motor/chase/go", &block_wait_drive);


void setup()

{
  
  follower.init(A2, A1, A0, &Saber);
  follower.setLimits(17,15,13);
  encoder.init();
 
  
  IR_R.init(A7);

  

 
 nh.initNode();
 nh.advertise(pub);
 nh.subscribe(sub);
 
 
  
  Saber.begin(2);

  
  
}
bool stopped = false;
bool published = false;
void loop() {
  if (stopped == false) {
    if (waiting == true) {
      nh.spinOnce();
    } else {
      if (go == true) {
        Saber.forward();
      }
      follower.update(Saber.FORWARD);
      go = false;
      
      if (positionFL >= 20000) {
        stopped = true;
        Saber.all_stop();
      }
    }
  } else {
    if (published == false) {
      pub.publish(&motor_state);
      published = true;
    }
  }
}
      
      
    
