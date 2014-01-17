/* WESLEY_SKETCH_MINI
 * written by: Eric M Gonzalez
 * date: 08-01-14
 *
 * PURPOSE: This sketch contains the response side of IR_CLASS_MEGA and
 *          resides on the mini.
 *
 *          REMEMBER TO MAKE SURE THAT THE SERIAL LINE-SPEED USED HERE
 *          MATCHES WHAT IS USED IN IR_CLASS_MEGA!
 */
#include "wesley_mini_bridge.h"
mini_bridge mega_br;

#include "gapFinder.h"
GapFinder gap_state;
// from mini-->mega

byte no_of_sensors;

void setup() {
	no_of_sensors = 3;
//	gap_state.init(A0, A1, A2);
	mega_br.begin(no_of_sensors);
}

void loop() {
	/* commented off for testing;
	if (gap_state.findGap() == gap_state.MAYBE_GAP) {
		mega_br.speed(mega_br.HALF_SPEED);
	} else
	if (gap_state.findGap() == gap_state.YES_GAP) {
		mega_br.speed(mega_br.FULL_STOP);
	} else
	if (mega_br.cmd_waiting()) {
		byte cmd = mega_br.cmd();
		switch(cmd) {
			case '$': {
				// package and send off an array of cm distances
				gap_state.report(mega_br.data_packet);
				mega_br.tunnel();
				break;
			}
			default: {
				break;
			}
		}
	}	*/

	delay(random(500, 1400));
	mega_br.speed(random(0,30));
//	Serial.print("mini: ");
//	Serial.print(random(0, 50), DEC);
//	Serial.println();
}
