/* WESLEY_IR_SKETCH_MEGA
 * written by: Eric M Gonzalez
 * date: 10-01-14
 *
 * PURPOSE: This sketch outlines the general order of events when using
 * 			the mega->mini bridge.
 */

#include "wesley_ir_class_mega.h"

// from mega-->mini
ir_bridge_mega mini_br;

void setup() {
}

void loop() {
	if (mini_br.ar_state == mega_br.AR_WAIT) {
		// using serial port 3 (pins 14(Tx) & 15(Rx))
		mini_br.begin(3);
		// or this, to supply initial sensor_count:
		//mini_br.begin(3, 8);
	}
	if (mega_br.peek() > 0) {
		// something on the serial line. process it!
		// add in motor commands.
	
	
}
