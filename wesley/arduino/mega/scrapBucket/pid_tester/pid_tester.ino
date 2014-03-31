#include <motor_cmd.h>
#include <QuadEncoder.h>
#include "RobotParams.h"
#include "TimeInfo.h"
#include "OdometricLocalizer.h"
#include "SpeedController.h"




/*
 *vicky wu 
 * trying to classify-integrate out Dr. Rainer Hessmer's pid code
 * umkc robotics 2014
 */


/*

shenanigans
.. the servos are saying 64. why. whyyyyyy
derp it's saying Ox40H - and it's doing a stop.





*/




QuadEncoder encoders;
motor_cmd sabertooth;


#define c_UpdateInterval 20  // update interval in milli seconds


// container for robot params wheel diameter [m], trackwidth [m], ticks per revolution
RobotParams _RobotParams = RobotParams();
TimeInfo _TimeInfo = TimeInfo();
OdometricLocalizer _OdometricLocalizer(&_RobotParams, &_TimeInfo);
SpeedController _SpeedController(&_OdometricLocalizer, &_RobotParams, &_TimeInfo);

void setup() {

  encoders.init();
  // put your setup code here, to run once:
  Serial.begin(9600);
  sabertooth.begin(2);


  _TimeInfo.Update();  
}

void loop() {
  // put your main code here, to run repeatedly: 
  //sabertooth.turn_right();
  //sabertooth.reverse_full();
  
  unsigned long milliSecsSinceLastUpdate = millis() - _TimeInfo.LastUpdateMillisecs;
  if(milliSecsSinceLastUpdate >= c_UpdateInterval)
  {
    //Serial.println(milliSecsSinceLastUpdate);
    // time for another update
    _TimeInfo.Update();
    //if (_IsInitialized)
    {
      DoWork();
    }
    //else
    {
      //RequestInitialization();
    }
  }
  
  // to sendvelocity command use this
  //_SpeedController.CommandVelocity(commandedVelocity, commandedAngularVelocity); 
  _SpeedController.CommandVelocity(5, 0); 
}

void DoWork()
{
  long _leftEncoderTicks = (encoders.getTicksFL() + encoders.getTicksBL())/2.0;
  long _rightEncoderTicks = (encoders.getTicksFR() + encoders.getTicksBR())/2.0;
  
  Serial.print("Ticks: ");
  Serial.print("\t");
  Serial.print(_leftEncoderTicks);
    Serial.print("\t");
  Serial.println(_rightEncoderTicks);
  
  
  _OdometricLocalizer.Update(_leftEncoderTicks, _rightEncoderTicks);
  //_BatteryMonitor.Update();
  _SpeedController.Update(false);  //don't worry about batery voltage for now
  IssueMotorCommands();  //boss the sabertooth motor controller aroun 
    
    
  /*  
  Serial.print("o"); // o indicates odometry message
  Serial.print("\t");
  Serial.print(_OdometricLocalizer.X, 3);
  Serial.print("\t");
  Serial.print(_OdometricLocalizer.Y, 3);
  Serial.print("\t");
  Serial.print(_OdometricLocalizer.Heading, 3);
  Serial.print("\t");
  Serial.print(_OdometricLocalizer.V, 3);
  Serial.print("\t");
  Serial.print(_OdometricLocalizer.Omega, 3);

/*
  Serial.print("\t");
  Serial.print(_OdometricLocalizer.VLeft, 3);
  Serial.print("\t");
  Serial.print(_OdometricLocalizer.VRight, 3);
  Serial.print("\t");
  Serial.print(_LeftEncoderTicks);
  Serial.print("\t");
  Serial.print(_RightEncoderTicks);
*/
  Serial.print("\n");

/*
  Serial.print("b\t"); // o indicates battery info message
  Serial.print(_BatteryMonitor.BatteryVoltage, 3);
  Serial.print("\t");
  Serial.print(_BatteryMonitor.VoltageIsTooLow);
  Serial.print("\n");
  */
}

void IssueMotorCommands()
{
  float normalizedRightMotorCV, normalizedLeftMotorCV;
  
  normalizedRightMotorCV = _SpeedController.NormalizedLeftCV;
  normalizedLeftMotorCV = _SpeedController.NormalizedRightCV;
  
  /*
  Serial.print("Speed: ");
  Serial.print(_SpeedController.DesiredVelocity);
  Serial.print("\t");
  Serial.println(_SpeedController.DesiredAngularVelocity);
  
  Serial.print("\t");
  Serial.print(_SpeedController.LeftError);
  Serial.print("\t");
  Serial.print(_SpeedController.RightError);
  Serial.print("\t");
  Serial.print(_SpeedController.TurnError);
  Serial.print("\t");
  */
  Serial.print("Normalized");
  Serial.print("\t");
  Serial.print(normalizedRightMotorCV);
  Serial.print("\t");
  Serial.print(normalizedLeftMotorCV);
  Serial.print("\n");
  
  
  //map float should work
  byte rightMotorValue = mapFloat(normalizedRightMotorCV, -1, 1, 0x00, 0x7F);     // scale it to use it with the servo (value 0x00 and 0x7F)
  byte leftMotorValue = mapFloat(normalizedLeftMotorCV, -1, 1, 0x00, 0x7F);     // scale it to use it with the servo (value 0x00 and 0x7F)
 
  
  Serial.print("Servos: ");
  Serial.print(rightMotorValue,HEX);
  Serial.print("\t");
  Serial.print(leftMotorValue,HEX);
  Serial.println("\n");
  
  //so i don't think this is the problem
  sabertooth.rightMotorCommand(rightMotorValue);
  sabertooth.leftMotorCommand(leftMotorValue);
}


byte mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  //my hard coding - assuming in_min = -1, in_max = 1...hardcoding 0 to output 0x40 - specific to our motor controller command for stop
  if(x == 0) return 0x40;
  return (byte) (  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min );
}

