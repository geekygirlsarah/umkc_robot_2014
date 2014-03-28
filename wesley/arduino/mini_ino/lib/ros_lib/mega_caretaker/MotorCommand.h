#ifndef _ROS_mega_caretaker_MotorCommand_h
#define _ROS_mega_caretaker_MotorCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mega_caretaker
{

  class MotorCommand : public ros::Msg
  {
    public:
      int8_t left;
      int8_t right;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_left;
      u_left.real = this->left;
      *(outbuffer + offset + 0) = (u_left.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->left);
      union {
        int8_t real;
        uint8_t base;
      } u_right;
      u_right.real = this->right;
      *(outbuffer + offset + 0) = (u_right.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_left;
      u_left.base = 0;
      u_left.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->left = u_left.real;
      offset += sizeof(this->left);
      union {
        int8_t real;
        uint8_t base;
      } u_right;
      u_right.base = 0;
      u_right.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->right = u_right.real;
      offset += sizeof(this->right);
     return offset;
    }

    const char * getType(){ return "mega_caretaker/MotorCommand"; };
    const char * getMD5(){ return "24825b8956c21f4c3dd28a5a4d09322c"; };

  };

}
#endif