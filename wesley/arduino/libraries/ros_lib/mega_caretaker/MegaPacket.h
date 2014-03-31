#ifndef _ROS_mega_caretaker_MegaPacket_h
#define _ROS_mega_caretaker_MegaPacket_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mega_caretaker/MotorCommand.h"

namespace mega_caretaker
{

  class MegaPacket : public ros::Msg
  {
    public:
      int8_t msgType;
      mega_caretaker::MotorCommand mCom;
      uint16_t payload;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_msgType;
      u_msgType.real = this->msgType;
      *(outbuffer + offset + 0) = (u_msgType.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->msgType);
      offset += this->mCom.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->payload >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->payload >> (8 * 1)) & 0xFF;
      offset += sizeof(this->payload);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_msgType;
      u_msgType.base = 0;
      u_msgType.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->msgType = u_msgType.real;
      offset += sizeof(this->msgType);
      offset += this->mCom.deserialize(inbuffer + offset);
      this->payload =  ((uint16_t) (*(inbuffer + offset)));
      this->payload |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->payload);
     return offset;
    }

    const char * getType(){ return "mega_caretaker/MegaPacket"; };
    const char * getMD5(){ return "70e1bad56237befa0ecfa4de7f08c859"; };

  };

}
#endif