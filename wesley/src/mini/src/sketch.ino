/* WESLEY_IR_SKETCH_MINI
 * written by: Eric M Gonzalez
 * date: 08-01-14
 *
 * PURPOSE: This sketch contains the response side of IR_CLASS_MEGA and
 *          resides on the mini.
 *
 *          REMEMBER TO MAKE SURE THAT THE SERIAL LINE-SPEED USED HERE
 *          MATCHES WHAT IS USED IN IR_CLASS_MEGA!
 */
#include "wesley_ir_class_mini.h"


// from mini-->mega
ir_bridge_mini mega_br;

void setup() {
}

void loop() {
	// this is wrong. start from the poll() and work out
	switch(mega_br.state) {
		case mega_br.WAITING {
			while(mega_br.begin() != mega_br.AR_GOOD);
			break;
		}
		case mega_br.CLOSING {
			// do non-intensive work to conserve power.
			break;
		}
		case mega_br.WORKING {
			// do main thrust of work here.
			byte cmd = mega_br.get_cmd()
			break;
		}
		default:
			break;

	// should be out of a WAITING state
}
