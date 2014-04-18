/*
umkc robotics 2014

Msg protocol for the msg MasterPacket in MegaCaretaker/msg.
Used to commmunicate from rest of board to the mega_caretaker node.
*/



#ifndef MASTER_MEGA_PACKET_DEFS
#define MASTER_MEGA_PACKET_DEFS


const static int8_t MASTER_MSGTYPE_COMMAND = 0;
const static int8_t MASTER_PL_GO_TO_TOOLS = 0;
const static int8_t MASTER_PL_CROSS_WAVES = 1;


const static int8_t MASTER_MSGTYPE_ACK = 1;
const static int8_t MASTER_PL_GO_TO_TOOLS_ACK = MASTER_PL_GO_TO_TOOLS;
const static int8_t MASTER_PL_CROSS_WAVES_ACK = MASTER_PL_CROSS_WAVES;

const static int8_t MASTER_MSGTYPE_STATE = 2;
const static int8_t MASTER_PL_GO_TO_TOOLS_FIN = MASTER_PL_GO_TO_TOOLS;
const static int8_t MASTER_PL_CROSS_WAVES_FIN = MASTER_PL_CROSS_WAVES;



#endif
