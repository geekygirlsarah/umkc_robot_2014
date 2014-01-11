/* WESLEY_MEGA_BRIDGE
 * written by: Eric M Gonzalez
 * date: 11-01-14
 *
 * PURPOSE: This class describes and defines the interface between
 * 			the mini and the mega.
 */

class mega_bridge {
	private:

	public:
		enum status { WAITING,
					  WORKING,
					  CLOSING };
		status state;
		 
		mega_bridge() {
			state = WAITING;
		};

        void begin(const byte port) {
        	switch(port) {
        		case 1:
        			#define mini Serial1
        			break;
        		case 2:
        			#define mini Serial2
        			break;
        		case 3:
        			#define mini Serial3
        			break;
        		default:
        			#define mini Serial3
        			break;
        	}
        	// YOU MUST MAKE SURE THAT THE LINE-SPEED MATCHES THE
        	// SPEED USED IN MINI_BRIDGE.
        	// Used as a literal to avoid mismatch.
            mini.begin(9600);
        }

		byte speed() {
			return(mini.read());
		}
		bool cmd_waiting() {
			return(mini.available() > 0);
		//	return(mini.peek() >= 0);
		}
		
		const byte cmd() {
			return(mini.read());
		}
};
