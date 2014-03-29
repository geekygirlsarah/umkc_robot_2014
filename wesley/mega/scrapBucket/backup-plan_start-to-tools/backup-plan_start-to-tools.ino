
//#include <MagellanEdit.h>

#include <Distance2D120X.h>

#include <DistSmoother.h>

#include <motor_cmd.h>

#include <QuadEncoder.h>

#include <WallFollower2.h>


#include <ros.h>
#include <std_msgs/Empty.h>


//char edge_flag = '_';

motor_cmd Saber;
//1 is forward,  2 is backward
//bool trouble; //trouble tells us when we're about to run off
//Distance2D120X IR_F;
//DistSmoother IR_F;

//
QuadEncoder encoder;
WallFollow follower;
DistSmoother IR_R;

//
//int avgFront = 0;

//int UPPER_LIMIT, LOWER_LIMIT, MIDDLE;
//bool notGood;


//float diffF_B;
//float diffM_B;
//float diffF_M;
////bool straight;
//int countExecute = 0;
//int count = 7;
//int flagCount = 0;
//int flagLeft = 0;
//int flagRight = 0;
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
  //IR_F.begin(A4);
  
  IR_R.init(A7);
  //IR_M.init();
  //R_F.init(A4);
  
  
  //LOWER_LIMIT = 13;
  //UPPER_LIMIT = 17;
  //MIDDLE = 15;
 
 nh.initNode();
 nh.advertise(pub);
 nh.subscribe(sub);
 
 
  
  Saber.begin(2);

  
  //A6 IS FRONT SO NORMALLY SWITCH THESE BUT MOTORS ARE DUMB
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
      
      
    
