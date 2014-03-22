#ifndef  REDUX_MEGA_PACKET_DEFS_H
#define REDUX_MEGA_PACKET_DEFS_H


	const static int8_t MSGTYPE_HEY = 0;

	const static int8_t PL_START_WAVE_CROSSING = 1;
	const static int8_t PL_START_TURNING_90 = 2;
	
	const static int8_t MSGTYPE_ACK = 1;
	const static int8_t PL_GENERAL_ACK = 0;
	const static int8_t PL_FINISHED_WAVE_CROSSING = PL_START_WAVE_CROSSING;
	const static int8_t PL_FINISHED_TURNING_90 = PL_START_TURNING_90;

	const static int8_t MSGTYPE_STATE = 2;
	const static int8_t PL_WAITING = 0;
	const static int8_t PL_START = 1;
	const static int8_t PL_TURNING_CW_INIT = 2;
	const static int8_t PL_TURNING_CW_WAIT = 3;
	const static int8_t PL_TURNING_CW_FIN = 4;
	const static int8_t PL_END = 5;


	const static int8_t MSGTYPE_MOTORCOM = 3;
	const static int8_t PL_STOP = 0;
	const static int8_t PL_TURNCW = 11;




#endif
