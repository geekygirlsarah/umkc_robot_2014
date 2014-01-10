/* WESLEY_IR_CLASS_MEGA
 * written by: Eric M Gonzalez
 * date: 08-01-14
 *
 * PURPOSE: This class defines and describes the MEGA interface to
 * 			the MINI attached via serial.
 */

#include "wesley_macros.h" 			// block_wait();

class ir_bridge_mega() {
	private:
		// maybe redo with VECOTRS via:
		// https://github.com/maniacbug/StandardCplusplus/
		// and then add data reading via:
		// http://stackoverflow.com/questions/5424042/class-variables-public-access-read-only-but-private-access-read-write

		// 0 - 255 centimeters (about 2.5 meters, or ~8 feet)
		// -128 - 127; even this is ~4 feet.
		// should be more than enough!
		// --- never trust that your data is sane.
		//
		// umm -- bytes are by definition unsigned. hrmm..
		// shorts?
		byte* data;

		// a whole, positive number. 
		// but, 255 sensors?
		byte no_of_sensors;

	public:
		enum ar_status { AR_GOOD, \
						 AR_WAIT, \
						 AR_EPORT, \
						 AR_ELINE }
		ar_status ar_state;

		ir_bridge_mega() { 
			ar_state = AR_WAIT;
			no_of_sensors = 0;
			data = NULL;
		};
		ir_bridge_mega(const byte serial_port) {
			ar_state = AR_WAIT;
			// currently undefined behavior in ir_class_mini if
			//    set_sensors is passed 0.
			no_of_sensors = 0;
			data = NULL;

			begin(serial_port);
		}
		// recommended constructor.
		ir_bridge_mega(const byte serial_port, const byte sensor_count) {
			ar_state = AR_WAIT;
			// currently undefined behavior in ir_class_mini if
			//    set_sensors is passed 0.
			no_of_sensors = 0;
			data = NULL;

			begin(serial_port);
			int error_code = set_sensor_count(sensor_count);
		}
		~ir_bridge_mega() {
			// owing to the Arduino's constant run and loop nature,
			//    this may not get called, but just in case the
			//    class instance is ever destroyed, prevent any
			//    memory leaks anyway.
			// since this destroys the data within the class, any
			//    external pointers to data will no longer function.
			//    undefined behavior.
			free(data);
		}
		
		// serial_port is one of [1, 2, 3]
		// this number represents which serial port this class
		//    can expect to use to communicate with the mini
		const byte begin(const byte serial_port) {
			switch (serial_port) {
				case '1':
					#define promini Serial1
					break;
				case '2':
					#define promini Serial2
					break;
				case '3':
					#define promini Serial3
					break;
				default:
					/* need better error-handling here if given an
					 * unexpected value. perhaps, return an EINVAL
					 * and print error codes on the serial line.
					 * this would allow the caller to verify that
					 * begin() worked, or failed and to try again. */
					// default to the USB serial connection
					//#define promini Serial
					break;
			}

			// this class takes care of starting the serial connection
			// so long as its not the USB connection
			#ifndef promini
				// THE LINE SPEED MUST MATCH THE IR_CLASS_MINI!
				// IT CANNOT BE ASSIGNED HERE AND PASSED ON.
				// Therefore, it is kept hidden to avoid mismatch.
				promini.begin(9600)
				// blocks until mini responds to initialization
				promini.write('S'); 	// SYN?
				ar_status = AR_GOOD;
				//while(promini.available() <= 0);
				block_wait(promini);
				if (promini.read() != 'A') {	// wait for 'ACK'
					ar_status = AR_ELINE;	// bad init across serial line
					promini.end(); 			// close the serial line
				}
			#elif
				// could not define promini ; failure
				ar_status = AR_EPORT;		// failure to assign a port
			#endif

			return(ar_status);
		}

		// set the number of sensors that will be attached.
		byte set_sensor_count(const byte sensor_count) {
			promini.write('%'); 		// % - assign sensor count
			no_of_sensors = sensor_count;
			promini.write(no_of_sensors);
			block_wait(promini);
			int return_count = promini.read();
			if (return_count != no_of_sensors) {
				// error
			}
			data = malloc(no_of_sensors);
		}
		const byte get_sensor_count() {
			return (no_of_sensors);
		}

		/// a bad way to write this. think gracefully.
		// retrieve the current data from the mini
		const byte* get_data() {
			promini.write('$'); 		// $ - collect IR data
			// after sending '$', wait until there is the expected
			// amount of data on the serial line. -- blocks
			// (probably blocks badly)
			while (promini.available() != no_of_sensors);
			for (int ith = 0; ith < no_of_sensors; ith++) {
				data[ith] = promini.read();
			}
			// return the pointer to the data array.
			return(data);
		}

		// retrieve gap state
:		byte get_gap_state() {
			promini.write('?'); 		// ? - retrieve gap_state
			//while(promini.available() <= 0);
			block_wait(promini);
			gap_state = promini.read();

		}

		// does a reset even make sense?
		// the IR sensor overwrites the data each cycle anyway.
		void reset() {
		//	promini.write('@'); 		// @ - sensor data reset
		};

		byte peek() {
			return(promini.peek());
		}
};
