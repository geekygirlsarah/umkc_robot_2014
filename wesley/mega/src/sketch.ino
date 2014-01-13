/* WESLEY_SKETCH_MEGA
 * written by: Eric M Gonzalez
 * date: 10-01-14
 *
 * PURPOSE: This sketch outlines the general order of events when using
 * 			the mega->mini bridge.
 */

#define console Serial

#include "wesley_mega_bridge.h"
mega_bridge mini_br;

#include "motor_cmd.h"
motor_cmd sabertooth;

void setup() {
	console.begin(9600);
	sabertooth.begin(2);
	mini_br.begin(3);
	console.println("ready");
}

void loop() {
	/*  commented for testing */
	if (mini_br.cmd_waiting() && \
		sabertooth.DIRECTION != sabertooth.STOPPED) {
	//	console.print("cmd from mini: ");
		const byte cmd = mini_br.cmd();
		delay(6);
		switch(cmd) {
			case '-':
			// commented off for testing
				if (sabertooth.DIRECTION == sabertooth.FORWARD) {
					sabertooth.forward(mini_br.cmd());
				} else
				if (sabertooth.DIRECTION == sabertooth.REVERSE) {
					sabertooth.reverse(mini_br.cmd());
				}
//				console.print(mini_br.cmd(), HEX);
				break;
			default:
				break;
		}
		console.println();
	//	delay(100);
	}	//*/

	/* comment out for testing */
	if (console.available() > 0) {
		// process incoming commands from console
		const byte cmd = console.read();
		switch(cmd) {
			case 'w':
				console.println("forward");
				sabertooth.forward(40);
			//	sabertooth.forward();
				break;
			case 's':
				console.println("reverse");
				sabertooth.reverse(40);
			//	sabertooth.reverse();
				break;
			case 'x':
			default:
				console.println("allstop");
				sabertooth.all_stop();
				break;
		}
	}	//*/

	//sabertooth.forward(40);
}
