#ifndef  REDUX_MEGA_PACKET_DEFS_H
#define REDUX_MEGA_PACKET_DEFS_H


	const static int8_t MSGTYPE_HEY = 0;

	const static int8_t PL_START_GO_TO_TOOLS = 0;
	const static int8_t PL_START_WAVE_CROSSING = 1;
	const static int8_t PL_START_TURNING_90_CW_X_AXIS = 2;
	const static int8_t PL_START_TURNING_90_CW_Y_AXIS = 3;
	const static int8_t PL_START_TURNING_90_CCW_X_AXIS = 4;
	const static int8_t PL_START_TURNING_90_CCW_Y_AXIS = 5;
	
        const static int8_t MSGTYPE_ACK = 5; 
        const static int8_t PL_GO_TO_TOOLS_ACK = PL_START_GO_TO_TOOLS;
	const static int8_t PL_WAVE_CROSSING_ACK = PL_START_WAVE_CROSSING;
	
	const static int8_t MSGTYPE_FINISHED = 1;
        const static int8_t PL_FINISHED_GO_TO_TOOLS = PL_START_GO_TO_TOOLS;
	const static int8_t PL_FINISHED_WAVE_CROSSING = PL_START_WAVE_CROSSING;
	const static int8_t PL_FINISHED_TURNING_90_CW_X_AXIS = PL_START_TURNING_90_CW_X_AXIS;
	const static int8_t PL_FINISHED_TURNING_90_CW_Y_AXIS = PL_START_TURNING_90_CW_Y_AXIS;
	const static int8_t PL_FINISHED_TURNING_90_CCW_X_AXIS = PL_START_TURNING_90_CCW_X_AXIS;
	const static int8_t PL_FINISHED_TURNING_90_CCW_Y_AXIS = PL_START_TURNING_90_CCW_Y_AXIS;

		/*
    const static int8_t PL_FINISHED_GO_TO_TOOLS = 10;
	const static int8_t PL_FINISHED_WAVE_CROSSING = 11;
	const static int8_t PL_FINISHED_TURNING_90_CW = 12;
	const static int8_t PL_FINISHED_TURNING_90_CCW = 13;
    */

	const static int8_t MSGTYPE_STATE = 2;
	const static int8_t PL_WAITING = 20;
	const static int8_t PL_LOOKING_FOR_GAP = 21;
	const static int8_t PL_TURNING_CW_INIT = 22;
	const static int8_t PL_TURNING_CCW_INIT = 23;
        const static int8_t PL_GAP_FOUND = 24;
    	const static int8_t PL_CROSSING_GAP = 25;
    	const static int8_t PL_GAP_CROSSED = 26;
	const static int8_t PL_FINDING_EDGE = 27;
        const static int8_t PL_TRANSITION_1_2 = 28;


	const static int8_t MSGTYPE_MOTORCOM = 3;
	const static int8_t PL_STOP = 30;
	const static int8_t PL_TURNCW = 31;
	const static int8_t PL_TURNCCW = 32;


	const static int8_t MSGTYPE_HANDSHAKE = 4;
	const static int8_t PL_SYN = 40;
	const static int8_t PL_SYN_ACK = 41;
	const static int8_t PL_ACK = 42;


#endif
