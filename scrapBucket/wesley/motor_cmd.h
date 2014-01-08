

class motor_cmd {
    private:
        // motor indentifiers - red button is front of vehicle
        const byte motor_R = 0x00;    // right side, as looking down
        const byte motor_L = 0x80;    // left side, as looking down
        
        // speed
        const byte ALL_STOP     = 0x00;
        const byte FULL_REVERSE = 0x01;
        const byte HALF_REVERSE = 0x10;
        const byte TURN_REVERSE = 0x30;
        const byte FULL_STOP    = 0x40;
        const byte TURN_FORWARD = 0x50;
        const byte HALF_FORWARD = 0x70;
        const byte FULL_FORWARD = 0x7F;

        // translate angle into a duration
        double burn_duration(int polar) {
            double arclength = ( polar / 360 ) * 2 * 3.14159265359;
            /* the constant 80.5555 comes from an assumption of:
                   1 second / revolution (1000 ms / rev)
                3600 ticks / revolution
                 290 ticks / inch        */
            double duration = 80.5555 * arclength;
        }

    public:
        // direction
        const byte FORWARD = 0xFF;
        const byte LEFT    = 0x33;
        const byte RIGHT   = 0x55;
        const byte REVERSE = 0xBB;
        const byte STOPPED = 0x00;
        byte DIRECTION;

        motor_cmd() {
            Serial2.begin(9600);
            DIRECTION = STOPPED;
        };
        // when calling movement commands, use and integer
        //   between 0  -- stop
        //   and    50  -- full tilt
        void forward( byte motor_speed ) {
            motor_speed += 77;
            Serial2.write(motor_L | motor_speed);
            Serial2.write(motor_R | motor_speed);
            DIRECTION = FORWARD;
        }            
        void reverse( long motor_speed ) {
            motor_speed = (((-66 / 50) * motor_speed) + 67);
            Serial2.write(motor_L | motor_speed);
            Serial2.write(motor_R | motor_speed);
            DIRECTION = REVERSE;
        }
        void turn_right( long alpha ) {
            Serial2.write(motor_L | HALF_FORWARD);
            Serial2.write(motor_R | HALF_REVERSE);
            DIRECTION = RIGHT;
            delay(burn_duration(alpha));
            Serial2.write(ALL_STOP);
            DIRECTION = STOPPED;
        }
        void turn_left( long alpha ) {
            Serial2.write(motor_L | HALF_REVERSE);
            Serial2.write(motor_R | HALF_FORWARD);
            DIRECTION = LEFT;
            delay(burn_duration(alpha));
            Serial2.write(ALL_STOP);
            DIRECTION = STOPPED;
        }
        void all_stop( ) {
            Serial2.write(ALL_STOP);
            DIRECTION = STOPPED;
        }
        
        byte state() {
            return DIRECTION;
        }
};

