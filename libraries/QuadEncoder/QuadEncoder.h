
/*
 * Encoder stuff
 * written by Darren Cogan dgc59c@mail.umkc.edu
 * umkc robotics 2014
 */


// Theoretical maximum speed of motor is 7840 CPS
// // This means 0.1276 ms per count. Or 2040 computer cycles per count!
//
// // Usable PCINT Ports on Arduino Mega:
// //   PORTB: PCINT[7:0]	  MegaPINS: 13:10, 50:53    Interrupt: PCI0
// //   PORTK: PCINT[23:16]  MegaPINS: Analog 15:8     Interrupt: PCI2 
//
// // Assuming it is wired up in this configuration
// // FR    BR    FL    BL
// // 13:12 11:10 50:51 52:53
// // A15   A13   A11   A9
//
// // We will use PORTB

#ifndef QUAD_ENCODER_H 
#define QUAD_ENCODER_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#elif defined(WIRING)
#include "Wiring.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif

volatile static uint8_t state;
volatile static uint8_t prev_state;
volatile static int32_t positionFR;
volatile static int32_t positionBR;
volatile static int32_t positionFL;
volatile static int32_t positionBL;


class QuadEncoder	{
	public:
		QuadEncoder()	{
		}

		void init()	{
			// Set all the port pins as INPUT
			// DDRx is the Data Direction Register for PORTx
			DDRK = 0x00;

			//??? Do our encoders need pull-up? NO
				PORTK = 0x00;

			// PCMSKx registers control which pins contribute to the interrupt
			// Set bit activates interrupt. x corresponds to PCIx
			// For our purposes we actually want to use the entire port
			PCMSK2 = 0xFF;

			// Now lets enable the interrupt
			PCICR |= B00000100; //last 3 bits enable the PCI2, PCI1, and PCI0 interrupts respectively

			//initialize state
			state = PINK;
			prev_state = state;

        //Activate Interrupts in general
        sei();
		

		}
		
		int32_t getTicksFR()	{ return positionFR;}
		int32_t getTicksBR()	{ return positionBR;}
		int32_t getTicksFL()	{ return positionFL;}
		int32_t getTicksBL()	{ return positionBL;}
	private:
		// Stuff to keep track of


};
// The ISR macro lets us actually define the code in our Interrupt Service Routine
// PCINT0_vect should point to memory address 0x0012 on the Atmega2560
ISR(PCINT2_vect)
{
	// encoder code goes here
	char tmp1;

	//get current state
	state = PINK;

        //Shuffle states
        tmp1 = (prev_state & B00110011) | ((state & B00110011) << 2);

	//compare them and update count
	switch (tmp1 & B00001111) {
		case 1: case 7: case 8: case 14:
			positionBL++;
			break;
		case 2: case 4: case 11: case 13:
			positionBL--;
			break;
		case 3: case 12:
			positionBL += 2;
			break;
		case 6: case 9:
			positionBL -= 2;
			break;
	}

	switch (tmp1 >> 4) {
		case 1: case 7: case 8: case 14:
			positionBR++;
			break;
		case 2: case 4: case 11: case 13:
			positionBR--;
			break;
		case 3: case 12:
			positionBR += 2;
			break;
		case 6: case 9:
			positionBR -= 2;
			break;
                default:
                        break;
	}

        tmp1 = ((prev_state & B11001100) >> 2) | (state & B11001100);
	switch (tmp1 & B00001111) {
		case 1: case 7: case 8: case 14:
			positionFL++;
			break;
		case 2: case 4: case 11: case 13:
			positionFL--;
			break;
		case 3: case 12:
			positionFL += 2;
			break;
		case 6: case 9:
			positionFL -= 2;
			break;
	}
	
	switch (tmp1 >> 4) {
		case 1: case 7: case 8: case 14:
			positionFR++;
			break;
		case 2: case 4: case 11: case 13:
			positionFR--;
			break;
		case 3: case 12:
			positionFR += 2;
			break;
		case 6: case 9:
			positionFR -= 2;
			break;
	}
        prev_state = state;
}

#endif
