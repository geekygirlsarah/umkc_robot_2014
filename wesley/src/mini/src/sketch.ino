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
	if (mega_br.ar_status == mega_br.AR_WAIT) {
		mega_br.begin();
	} else {
		mega_br.sendcmd(gapFinder.findGap());
		;w
}
