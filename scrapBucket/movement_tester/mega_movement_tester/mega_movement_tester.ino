


/* gap finder tester 
 * written by: victoria wu
 * date: 1/17/14 
 *
 * what: make the robot keep going until the gap is found. then stop. (going to put it all on the mega, no bridge just yet
 * FIRST - just getting the robot to move. 
 * PURPOSE: 
 
 * .... pro tip: if you get the error "serial2 is not declared in this scope" check that the board you're programming for IS THE MEGA
 */


#include <motor_cmd.h>
#include <wesley_mega_bridge.h>

#include <Encoder.h>
#define console Serial



mega_bridge mini_br;
motor_cmd sabertooth;
//Encoder myEnc(A10, A11);
//long oldPosition  = -999;

void setup() {
	console.begin(9600);
	sabertooth.begin(2);
	mini_br.begin(3);
	console.println("ready");
}

void loop() {
	/* comment out for testing */

       
       
      /*
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

	}	
*/


        //one problem. since the sabertooth assumes starting as stopped, you can never go forward becasue it checks if the direction is forward.
        if (mini_br.cmd_waiting() ) {
                //sabertooth.DIRECTION != sabertooth.STOPPED) {
                console.print("cmd from mini: ");
                console.println();
                byte cmd = mini_br.cmd();
                console.println(cmd,DEC);
                delay(6);
                switch(cmd) {
                        case '-':  //speed command
                                console.println("Speed Command!");
                                delay(10);
                                cmd = mini_br.cmd();
                                console.println(cmd);
                                //sabertooth.forward(cmd);
                                
                                //if (sabertooth.DIRECTION == sabertooth.FORWARD) {
                                 //       console.print("go forwards");
                                  //      sabertooth.forward(mini_br.cmd());
                               // } else
                                //if (sabertooth.DIRECTION == sabertooth.REVERSE) {
                                //        console.print("go backwards");
                                 //       sabertooth.reverse(mini_br.cmd());
                                //}
                                
                                //console.print(mini_br.cmd(), HEX);
                                
                                break;
                        default:
                                console.print("what is this?");
                                console.print(cmd,DEC);
                                //sabertooth.all_stop();   
                                break;
                }
                console.println();
                delay(100);
        }        
       
        
        
	//sabertooth.forward(40);
        
        /*
        long newPosition = myEnc.read();
        if (newPosition != oldPosition) {
            oldPosition = newPosition;
            Serial.println(newPosition);
        }
        */
}
