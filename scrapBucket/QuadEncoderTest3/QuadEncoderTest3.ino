#include <motor_cmd.h>
#include <QuadEncoder.h>

/*Encoder stuff
* written by Darren Cogan
* UMKC robotics 2014
*/
QuadEncoder encoders;
motor_cmd sabertooth;


void setup() {

  encoders.init();
  // put your setup code here, to run once:
  Serial.begin(9600);
  sabertooth.begin(2);      
}

void loop() {
  // put your main code here, to run repeatedly: 
  sabertooth.turn_right(360);
  
  Serial.print("FL: ");
  Serial.print(encoders.getTicksFR());
  /*
  Serial.print(" BL: ");
  Serial.print(positionBL);
  Serial.print(" FR: ");
  Serial.print(positionFR);
  Serial.print(" BR: ");
  Serial.println(positionBR);
  digitalWrite(13, LOW);  //whats this for? (TODO ask darren)
  Serial.println(PINK);    //whats this for?
  */
}


