/* arm_control.h
 * written by: Eric Gonzalez, Sarah Withee
 * date: 2014-01-18
 *
 * PURPOSE: Define and describe a control interface for the robotic arm
 *
 * TODO:
 * - give [Eric] a visual representation of the arms co-ordinate frame; which 
 *   way are the axes oriented. From this, I should be able to position the claw 
 *   as needed by issuing simple move_to(x, y, z, p) calls.
 *   (This is in person, not in code, but we still need to do it)      
 * - alter put(byte, byte) and p_put(byte, short) to work correctly.
 * - add in angle limits so that if anything goes beyond the range of the 
 *   allowed pulse widths it defaults to the min or max pulse width 
 *   appropriately.
 * - need a forward kinematic.
 *   -- sadly, this [Eric] did not add this in to the re-written code so you'll 
 *      need to find an old copy.
 * - Move internal 'helper' functions to private area
 * - Remove extraneous commented out code
 * - Make any 'magic' numbers precompiler constants for optimization
 *     
 */


// Remove Arduino.h if compiling with gcc or avr-gcc. Leave in if compiling
// with Arduino IDE
#include <Arduino.h>
#include <Servo.h>

class arm_control {
	private:
		byte no_of_joints;

        // Dynamic array of servos
		Servo* arm;

		short* p_destination;
		short* p_position;

	public:
        enum JOINTS { BASE, SHOULDER, ELBOW, WRIST_P, WRIST_R, HAND };
        struct point {
    		int x, y, z;
    		point(int x = 0, int y = 0, int z = 0) :
    			x(x), y(y), z(z) { };
    	};

        // Constructor/deconstructor
		arm_control();
		arm_control(byte joints);
		~arm_control();

        // Initializing functions
		void connect(byte argc, ...);
		byte read(const byte joint);
		void initial_park();			

        // Exact placement functions
		void park();
		void carry();

        // Exact movement functions
		void putPulse(const byte joint, const short pulse);
		void putAngle(const byte joint, const byte angle);
    	void update();

        // Relative movement functions
		void move_to( float x, float y, float z, float grip_wrist, float grip_rotate);
        struct point getXYZ() {
};

// EOF

