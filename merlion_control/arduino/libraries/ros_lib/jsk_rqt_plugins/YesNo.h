#ifndef _ROS_SERVICE_YesNo_h
#define _ROS_SERVICE_YesNo_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jsk_rqt_plugins
{

static const char YESNO[] = "jsk_rqt_plugins/YesNo";

  class YesNoRequest : public ros::Msg
  {
    public:

    YesNoRequest()
    {
    }

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

    const char * getType(){ return YESNO; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class YesNoResponse : public ros::Msg
  {
    public:
      typedef bool _yes_type;
      _yes_type yes;

    YesNoResponse():
      yes(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_yes;
      u_yes.real = this->yes;
      *(outbuffer + offset + 0) = (u_yes.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->yes);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_yes;
      u_yes.base = 0;
      u_yes.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->yes = u_yes.real;
      offset += sizeof(this->yes);
     return offset;
    }

    const char * getType(){ return YESNO; };
    const char * getMD5(){ return "aa7d186fb6268a501cd4c0c274f480ff"; };

  };

  class YesNo {
    public:
    typedef YesNoRequest Request;
    typedef YesNoResponse Response;
  };

}
#endif
