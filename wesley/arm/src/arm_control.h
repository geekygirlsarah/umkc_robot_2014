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

        // Constructor
		arm_control();

        // Parameterized constructor
		arm_control(byte joints);

        // Deconstructor
		~arm_control();

        // Attaches a set of pins to the servos
		void connect(byte argc, ...);

        // This needs to be called immediately after connect.
		// this sets the arm into a sane, locked position.
		// these values can be adjusted as necessary.
		void initial_park();			

        // Puts the arm in a good resting (parked) position
		void park();

        // This puts the arm in a good position to carry a tool as the bot 
		// drives around. Wrist will point up.
		void carry();

        // Returns the angle value of the servo
		byte read(const byte joint);

        // Directly write a pulse to the servo, would take the place of the 
		// function put (byte, byte)         
		void p_put(const byte joint, short pusle);

        // Directly put the given joint to the given angle,
		// converting first to pulse width
		void put(const byte joint, const byte angle);

        // Call update with update(NO_OF_JOINTS, <a list of joints to mov
    	void update(const byte argc, ...);
        
        // Places at (x, y, z) - this is an inverse kinematic
		//    equation that translates the (x, y, z) into
		//    angualr measures from an origin defined at the
		//    base of the arm.
		void move_to( float x, float y, float z, float grip_angle_d, bool moveSmooth = true);
};

// EOF

