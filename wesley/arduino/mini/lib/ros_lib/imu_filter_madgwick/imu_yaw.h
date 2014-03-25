#ifndef _ROS_SERVICE_imu_yaw_h
#define _ROS_SERVICE_imu_yaw_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace imu_filter_madgwick
{

static const char IMU_YAW[] = "imu_filter_madgwick/imu_yaw";

  class imu_yawRequest : public ros::Msg
  {
    public:

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return IMU_YAW; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class imu_yawResponse : public ros::Msg
  {
    public:
      float yaw;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      int32_t * val_yaw = (int32_t *) &(this->yaw);
      int32_t exp_yaw = (((*val_yaw)>>23)&255);
      if(exp_yaw != 0)
        exp_yaw += 1023-127;
      int32_t sig_yaw = *val_yaw;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_yaw<<5) & 0xff;
      *(outbuffer + offset++) = (sig_yaw>>3) & 0xff;
      *(outbuffer + offset++) = (sig_yaw>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_yaw<<4) & 0xF0) | ((sig_yaw>>19)&0x0F);
      *(outbuffer + offset++) = (exp_yaw>>4) & 0x7F;
      if(this->yaw < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t * val_yaw = (uint32_t*) &(this->yaw);
      offset += 3;
      *val_yaw = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_yaw |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_yaw |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_yaw |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_yaw = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_yaw |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_yaw !=0)
        *val_yaw |= ((exp_yaw)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->yaw = -this->yaw;
     return offset;
    }

    const char * getType(){ return IMU_YAW; };
    const char * getMD5(){ return "08cb8274b6ddb17af5a842bca0b17db1"; };

  };

  class imu_yaw {
    public:
    typedef imu_yawRequest Request;
    typedef imu_yawResponse Response;
  };

}
#endif
