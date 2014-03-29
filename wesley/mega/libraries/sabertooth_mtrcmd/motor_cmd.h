/* MOTOR_CMD
 * written by: Eric M Gonzalez
 * date: 30-12-13 (or thereabouts)
 *
 * PURPOSE: This file defines and describes an interface to the
 * 			sabertooth motor controller.
 */

#ifndef MOTOR_CMD_H
#define MOTOR_CMD_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include <pins_arduino.h>
#endif

class motor_cmd {
    private:
        // motor indentifiers - red button is front of vehicle
        static const byte motor_R = 0x80;    // right side, as looking down
        static const byte motor_L = 0x00;    // left side, as looking down
        
        // speed
        static const byte ALL_STOP     = 0x00;
        static const byte FULL_FORWARD = 0x01;
	static const byte FULL_FORWARD_LEFT = 0x03;
	static const byte FULL_FORWARD_RIGHT = 0x01;
        static const byte HALF_FORWARD = 0x10;
        static const byte QUARTER_FORWARD = 0x20;
	static const byte QUARTER_FORWARD_LEFT = 0x23;
	static const byte QUARTER_FORWARD_RIGHT = 0x20;
	static const byte QUARTER_FORWARD_LEFT_COURSECORRECT_LEFT = 0x24;
	static const byte QUARTER_FORWARD_LEFT_COURSECORRECT_RIGHT = 0x20;

	static const byte QUARTER_FORWARD_RIGHT_COURSECORRECT_RIGHT = 0x27;	 
	static const byte QUARTER_FORWARD_RIGHT_COURSECORRECT_LEFT = 0x21;


        static const byte TURN_FORWARD = 0x30;
        static const byte FULL_STOP    = 0x40;
        static const byte TURN_REVERSE = 0x50;
        static const byte QUARTER_REVERSE = 0x60;
	static const byte QUARTER_REVERSE_RIGHT = 0x5E;
	static const byte QUARTER_REVERSE_LEFT = 0X60;
	static const byte QUARTER_REVERSE_COURSECORRECT_LEFT = 0x5F;   //USE THIS WHEN WANTING TO TURN LEFT
	static const byte QUARTER_REVERSE_COURSECORRECT_RIGHT = 0x5D; //USE THIS WHEN WANTING TO TURN RIGHT
        static const byte HALF_REVERSE = 0x70;
        static const byte FULL_REVERSE = 0x7F;
	 

        // translate angle into a duration
        // the calculations here need better work. find out how
        //    fast to turn the robot, and then calculate time from
        //    there.
        // or, find out what speed 1 second / revolution is.
        double burn_duration(int polar) {
            double arclength = ( polar / 360 ) * 2 * 3.14159265359;
            /* the constant 80.5555 comes from an assumption of:
                   1 second / revolution (1000 ms / rev)
                3600 ticks / revolution
                 290 ticks / inch        */
            double duration = 80.5555 * arclength;
        }

    public:
        // direction
        static const byte FORWARD = 0xFF;
        static const byte LEFT    = 0x33;
        static const byte RIGHT   = 0x55;
        static const byte REVERSE = 0xBB;
        static const byte STOPPED = 0x00;
        byte DIRECTION;

        motor_cmd() {
            DIRECTION = STOPPED;
        };

        void begin(const byte serial_port) {
			switch(serial_port) {
				case 1:
					#define mcontrol Serial1
					Serial.print("Serial1: ");
					break;
				case 2:
					#define mcontrol Serial2
					Serial.print("Serial2: ");
					break;
				case 3:
					#define mcontrol Serial3
					Serial.print("Serial3: ");
					break;
				default:
					#define mcontrol Serial2
					Serial.print("Serial2: ");
					break;
			}

			mcontrol.begin(9600);
        }

		//pid shenanigans. 
		//send it something from 0x00 to 0x7F for speed and direction (TODO look at provided pid code.. how to account for direction?)
		void rightMotorCommand(byte motor_speed)	{
            mcontrol.write(motor_R | motor_speed);
			if(motor_speed < 0x40)
					DIRECTION = REVERSE;
			else
					DIRECTION = FORWARD;
		}

		void leftMotorCommand(byte motor_speed)	{
            mcontrol.write(motor_L | motor_speed);
			if(motor_speed < 0x40)
					DIRECTION = REVERSE;
			else
					DIRECTION = FORWARD;
		}
        // when calling movement commands, use and integer
        //    between 0  -- stop
        //    and    50  -- full tilt
        /* the forward() and reverse() functions will translate the
         *    range of 0 - 50 to an actual range of 0x01 - 0x70.
         *
         *       |
         *   50 -|               
         *       | \                           /
         *       |   \                       /
         *       |     \                   /
         *       |       \               /
         *       |         \           /
         *       |           \       /
         *       |             \   /
         *    0 -|---------------|---------------|
         *      0x00            0x40            0x7F
         *
         *      REV <-- faster  STOP  faster --> FWD
         *
         * The way that the sabertooth controls a motor is as a byte
         *   from 0x00 through 0xFF, with the MSB indicating which
         *    side of the two motor-controllers to control. This leaves
         *    bits ?7654321 (0x00 - 0x7F) to indicate both speed and
         *    direction.
         *       0x00 - 0x39 are reverse, lower is faster
         *   	        0x40 is stop
         *       0x41 - 0x7F are forward, higher is faster
         * The ranges from either side of 0x40 are linear, but with
         *    differing slopes. by extrapolating (y = mx + b) with:
         *
         *    forward: y = 1.26x+64   reverse: y = -1.26x+64
         *       actual   parameter      actual   parameter
         *         64   =     0             1  =     50
         *        127   =    50            64  =      0
         *
         *    results in the above functions.
         */
        void forward( byte motor_speed ) {
        	// make a conversion from supplied range to the actual
        	//Serial.print("   ::CMD:: forward - at value: ");
			//Serial.print(motor_speed, DEC); Serial.print(" : ");
            motor_speed = ( 1.26)*motor_speed + 64;
            Serial.println(motor_speed, DEC);
            mcontrol.write(motor_L | motor_speed);
            mcontrol.write(motor_R | motor_speed);
            DIRECTION = FORWARD;
        }            
        void reverse( byte motor_speed ) {
        	//Serial.print("   ::CMD:: reverse - at value: ");
			//Serial.print(motor_speed, DEC); Serial.print(" : ");
            motor_speed = (-1.26)*motor_speed + 64;
//            Serial.println(motor_speed, DEC);
            mcontrol.write(motor_L | motor_speed);
            mcontrol.write(motor_R | motor_speed);
            DIRECTION = REVERSE;
        }

        void forward() {
        	// make a conversion from supplied range to the actual
        	Serial.println("   ::CMD:: forward - at HALF_ value");
            mcontrol.write(motor_L | QUARTER_FORWARD_LEFT);
            mcontrol.write(motor_R | QUARTER_FORWARD_RIGHT);
            DIRECTION = FORWARD;
}
 	           
        void reverse() {
        	Serial.println("   ::CMD:: reverse - at HALF_ value");
            mcontrol.write(motor_L | QUARTER_REVERSE_LEFT);
            mcontrol.write(motor_R | QUARTER_REVERSE_RIGHT);
            DIRECTION = REVERSE;
        }
	void reverse_correct_left() {
		Serial.println("   ::CMD:: correcting to the left");
	    mcontrol.write(motor_L | QUARTER_REVERSE_COURSECORRECT_LEFT);
	    mcontrol.write(motor_R | QUARTER_REVERSE_COURSECORRECT_RIGHT);
	}

	void reverse_correct_right() {
		Serial.println("   ::CMD:: correcting to the right");
	    mcontrol.write(motor_L | QUARTER_REVERSE_COURSECORRECT_LEFT);
	    mcontrol.write(motor_R | QUARTER_REVERSE_COURSECORRECT_RIGHT);
	}

	void forward_correct_right() {
		Serial.println("   ::CMD:: correcting to the right");
	    mcontrol.write(motor_L | QUARTER_FORWARD_RIGHT_COURSECORRECT_LEFT);
	    mcontrol.write(motor_R | QUARTER_FORWARD_RIGHT_COURSECORRECT_RIGHT);
	}
	
	void forward_correct_left() {
		Serial.println("   ::CMD:: correcting to the left");
	    mcontrol.write(motor_L | QUARTER_FORWARD_LEFT_COURSECORRECT_LEFT);
	    mcontrol.write(motor_R | QUARTER_FORWARD_LEFT_COURSECORRECT_RIGHT);
	}
	
		//turn right forever
		void turn_right(byte motor_speed) {
           	Serial.print("   ::CMD:: turn right- at value hex: ");
			motor_speed = ( 1.26)*motor_speed + 64;
			Serial.print(motor_speed, HEX); Serial.print(" : ");
            mcontrol.write(motor_L | motor_speed);
            motor_speed = motor_speed - ( 2*(motor_speed-0x40)); 
			Serial.println(motor_speed, HEX); Serial.print(" : ");
            mcontrol.write(motor_R | motor_speed);
            DIRECTION = RIGHT;
        }
 		//turn left forever	
		void turn_left( byte motor_speed) {
            motor_speed = ( 1.26)*motor_speed + 64;
            mcontrol.write(motor_R | motor_speed);
            motor_speed = ( -1.26)*motor_speed + 64;
            mcontrol.write(motor_L | motor_speed);
            DIRECTION = LEFT;
        }

		/* just turn forever at half speed. no setting the speed 
         */
        
		void turn_right( ) {
            mcontrol.write(motor_L | HALF_FORWARD);
            mcontrol.write(motor_R | HALF_REVERSE);
            DIRECTION = RIGHT;
        }
        void turn_left( ) {
            mcontrol.write(motor_L | HALF_REVERSE);
            mcontrol.write(motor_R | HALF_FORWARD);
            DIRECTION = LEFT;
        }

		void forward_full( ) {
            mcontrol.write(motor_L | FULL_REVERSE);
            mcontrol.write(motor_R | FULL_REVERSE);
            DIRECTION = FORWARD;
        }
        void reverse_full( ) {
            mcontrol.write(motor_L | FULL_FORWARD);
            mcontrol.write(motor_R | FULL_FORWARD);
            DIRECTION = REVERSE;
        }




        /* here, we're using angle alpha as a turning reference.
         *    the angle supplied can be arbitrarily large. why
         *    it's a long is ridiculous.
         */
        /*
		void turn_right( long alpha ) {
            mcontrol.write(motor_L | HALF_FORWARD);
            mcontrol.write(motor_R | HALF_REVERSE);
            DIRECTION = RIGHT;
            delay(burn_duration(alpha));
            mcontrol.write(ALL_STOP);
            DIRECTION = STOPPED;
        }
        void turn_left( long alpha ) {
            mcontrol.write(motor_L | HALF_REVERSE);
            mcontrol.write(motor_R | HALF_FORWARD);
            DIRECTION = LEFT;
            delay(burn_duration(alpha));
            mcontrol.write(ALL_STOP);
            DIRECTION = STOPPED;
        }
		*/
        void all_stop( ) {
        	Serial.println("   ::CMD:: stopping");
            mcontrol.write(ALL_STOP);
            DIRECTION = STOPPED;
        }

		void turnCCW()	{
			turn(QUARTER_REVERSE, QUARTER_FORWARD);
//			turn(HALF_REVERSE, HALF_FORWARD);
		}

		void turnCW()	{
			turn(QUARTER_FORWARD, QUARTER_REVERSE);
//			turn(HALF_FORWARD, HALF_REVERSE);
		}
        
		void turn(byte right, byte left)	{
					Serial.println("movement::forever turn");
					rightMotorCommand(right);
					leftMotorCommand(left);
				
				}

        byte state() {
            return DIRECTION;
        }
};

#endif
