/* WESLEY_IR_CLASS_MINI
 * written by: Eric M Gonzalez
 * date: 08-01-14
 *
 * PURPOSE: This class contains the response side of IR_CLASS_MEGA and
 *          resides on the mini.
 *
 *          REMEMBER TO MAKE SURE THAT THE SERIAL LINE-SPEED USED HERE
 *          MATCHES WHAT IS USED IN IR_CLASS_MEGA!
 */

#include "wesley_macros.h"

#include <DistanceGP2Y0A21YK>
#define mega Serial

class ir_bridge_mini() {
	private:
		byte no_of_sensors;
		byte* distance_cm;
		byte* package_out;
		long* distance_raw;
		DistanceGP2Y0A21YK* sensors_array;

	public:
		enum ar_status { AR_GOOD, \
						 AR_WAIT, \
						 AR_EPORT, \
						 AR_ELINE }
		ar_status ar_state;
		enum status { WAITING, \  // waiting for setup from IR_CLASS_MEGA
					  WORKING, \  // processing
					  CLOSING };  // standby\shutdown state
		status state;

		ir_bridge_mini() {
			state = WAITING;
			ar_state = AR_WAIT;
			// only make the empty data sane.
			// DO NOT USE BEGIN IN THE CONSTRUCTOR!
			no_of_sensors = 0;
			distance_cm = NULL;
			distance_raw = NULL;
			sensors_array = NULL;
			package_out = NULL;
		}
		~ir_bridge_mini() {
			// if for some reason this class is destroyed, clean up
			//    any allocations to prevent memory leaks.
			// since this destroys the internal data, any external
			//   copies of the pointer will be useless.
			//   undefined behavior.
			free (distance_cm);
			free (distance_raw);
			free (sensors_array);
		}

		const byte begin() {
			mega.begin(9600);

			ar_status = AR_GOOD;
			mega.write('S'); 		// SYN?
			block_wait(mega);
			if (mega.read() != 'A') {
				ar_status = AR_ELINE; 	// bad init on serial line.
				mega.end(); 			// close the serial line
			}
			return(ar_status);
		}

		// set sensor array size
		// returns the number of sensors initialized.
		// TODO: need to handle being passed 0.
		byte set_sensor_count(const byte sensor_count) {
			no_of_sensors = sensor_count;
			// this is lame right now. should be more robust and
			//    actually mean something. it'd be cool if the
			//    status of a line can be checked BEFORE this
			//    function leaves.

			// really should wrap all of these mallocs in their own
			//    error checking if statements.
			// i'll save that for later.
			distance_cm = malloc(no_of_sensors);
			distance_raw = malloc(no_of_sensors);
			sensors_array = malloc(no_of_sensors);
			package_out = malloc(no_of_sensors);

			return(no_of_sensors);
		}
		byte get_sensor_count() {
			return(no_of_sensors);
		}

		const byte get_cmd() {
			block_wait(mega);
			return(mega.read());
		}

		// sends CM!, not RAW
		const byte* send_data() {
			// update distance_cm through function call
			// make a copy of it, and pass it back.
			memcpy(package_out, distance_cm, no_of_sensors);
			return(package_out);
		}
