#ifndef _ROS_wesley_arm_angle_h
#define _ROS_wesley_arm_angle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace wesley
{

  class arm_angle : public ros::Msg
  {
    public:
      uint16_t base;
      uint16_t shoulder;
      uint16_t elbow;
      uint16_t wrist_pitch;
      uint16_t wrist_roll;
      uint16_t hand;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->base);
      *(outbuffer + offset + 0) = (this->shoulder >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->shoulder >> (8 * 1)) & 0xFF;
      offset += sizeof(this->shoulder);
      *(outbuffer + offset + 0) = (this->elbow >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->elbow >> (8 * 1)) & 0xFF;
      offset += sizeof(this->elbow);
      *(outbuffer + offset + 0) = (this->wrist_pitch >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->wrist_pitch >> (8 * 1)) & 0xFF;
      offset += sizeof(this->wrist_pitch);
      *(outbuffer + offset + 0) = (this->wrist_roll >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->wrist_roll >> (8 * 1)) & 0xFF;
      offset += sizeof(this->wrist_roll);
      *(outbuffer + offset + 0) = (this->hand >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->hand >> (8 * 1)) & 0xFF;
      offset += sizeof(this->hand);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->base =  ((uint16_t) (*(inbuffer + offset)));
      this->base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->base);
      this->shoulder =  ((uint16_t) (*(inbuffer + offset)));
      this->shoulder |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->shoulder);
      this->elbow =  ((uint16_t) (*(inbuffer + offset)));
      this->elbow |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->elbow);
      this->wrist_pitch =  ((uint16_t) (*(inbuffer + offset)));
      this->wrist_pitch |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->wrist_pitch);
      this->wrist_roll =  ((uint16_t) (*(inbuffer + offset)));
      this->wrist_roll |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->wrist_roll);
      this->hand =  ((uint16_t) (*(inbuffer + offset)));
      this->hand |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->hand);
     return offset;
    }

    const char * getType(){ return "wesley/arm_angle"; };
    const char * getMD5(){ return "ef33d4984a305fd75dffb44db14b17ea"; };

  };

}
#endif