/* wesley prototyping - written by:
   Vicky, Eric, and Andrew (#5);  */

#include "Arduino.h"
#include "motor_cmd.h"
   
// encoders
//#define ENCODER_OPTIMIZE_INTERRUPTS
//#include <Encoder.h>
//Encoder enc_R(18, 19);
//Encoder enc_L(20, 21);

// IR sensors
#include <DistanceGP2Y0A21YK.h>
DistanceGP2Y0A21YK Dist1;
DistanceGP2Y0A21YK Dist2;
int distance1;
int distance2;


#define console Serial


// gap finding data
int check = 3;        //how many times in a row must the two sensors read a hole
int threshold = 20;   //threshold for ping sensor detecting a hole (cm)
enum ternary  { no_gap, maybe_gap, yes_gap };
ternary gap_status = no_gap;    //we start out assuming no hole
bool gapPresent()  {
  return Dist1.isFarther(threshold) && Dist2.isFarther(threshold);
}

// button
const byte button = 8;
boolean readyToGo = false;
void goButtonWait() {
    console.print(".");
    while(digitalRead(button));
    readyToGo = true;
    console.println();
    console.println("MOVING!");
}


// rename the serial ports through defines to something
//   that shows the direction of the communication.
motor_cmd sabertooth;
void setup() {
    console.begin(9600);
    Dist1.begin(A1);
    Dist2.begin(A2);
    
    pinMode(button, INPUT);
    digitalWrite(button, HIGH);
}

void loop() {
    if (!readyToGo) {
        goButtonWait();  // blocks until button is pressed
        delay(1500);    // wait 1.5 seconds before moving.
    }
//    if (sabertooth.DIRECTION == sabertooth.STOPPED) {
//        sabertooth.forward(40);
//    }
//    if (sabertooth.DIRECTION == sabertooth.FORWARD) {
//        delay(1500);
//        sabertooth.all_stop();
//    }
    sabertooth.forward(40);
    delay(1500);
    sabertooth.all_stop();
    delay(500);
    sabertooth.turn_left(90);
    delay(500);
    sabertooth.turn_right(180);
    delay(500);
    sabertooth.turn_left(90);
    delay(500);
    sabertooth.reverse(40);
    delay(1500);
    sabertooth. all_stop();
}

long trips[2][5] = { { 0, 0, 0, 0, 0 },
                     { 0, 0, 0, 0, 0 } };
//long trips[2][5];
void update_trips(long distance_left, long distance_right) {
    for ( int jth = 0; jth < 1; jth++) {
        for ( int ith = 0; ith < 4; ith++ ) {
            trips[jth][ith+1] = trips[jth][ith];
        }
    }
    trips[0][0] = distance_right;
    trips[1][0] = distance_left;
}

