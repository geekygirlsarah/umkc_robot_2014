/* wesley_mini_bridge.h
 * written by: Eric M Gonzalez
 * date: 11-01-14
 *
 * lives on the mini. allows mini to commnunicate with mega
 */

#ifndef WESLEY_MINI_BRIDGE_H
#define WESLEY_MINI_BRIDGE_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include <pins_arduino.h>
#endif

class mini_bridge {
	private:
		byte no_of_sensors;
	
	public:
		enum status { WAITING,
					  WORKING,
					  CLOSING };
		status state;
		#define mega Serial
		static const byte FULL_STOP  = 0x00;
		static const byte HALF_SPEED = 0x30;
		byte* data_packet;

		mini_bridge() {
			state = WAITING;
			no_of_sensors = 0;
			data_packet = NULL;
		};

		~mini_bridge() {
			delete (data_packet);
		}

		// count is the number of IR sensors being tracked
		// YOU MUST MAKE SURE THAT THE LINE-SPEED MATCHES THE LINE-
		// SPEED USED IN THE MEGA_IR BRIDGE. THIS CANNOT BE SET HERE
		// AND PASSED ON.
		// Defined as constant and not accessible to prevent mis-match.
		void begin(const byte count) {
			mega.begin(9600);
			state = WORKING;
			no_of_sensors = count;
		//	data_packet = malloc(count);
			data_packet = new byte[count];
		}

		// range:  0 - stop
		// 		  50 - full
		void speed(const byte value) {
			byte cmd[2] = { 0, 0 };
			cmd[0] = '-';
			cmd[1] = value;
			mega.write(cmd, sizeof(cmd));
		//	mega.write('-');
		//	mega.write(value);
		//	mega.print(value, DEC);

		//  mega.flush() ??
		}

		bool cmd_waiting() {
			// take a look and see if there something waiting
			return(mega.peek() >= 0);
		}

		const byte cmd() {
			// read one byte and return it
			return(mega.read());
		}

		void tunnel() {
			// inform the receving end how much data to expect
			mega.write(no_of_sensors);
			// send the data across
			mega.write(data_packet, no_of_sensors);
		}
};

#endif
