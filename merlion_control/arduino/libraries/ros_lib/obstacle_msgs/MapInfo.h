#ifndef _ROS_obstacle_msgs_MapInfo_h
#define _ROS_obstacle_msgs_MapInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "obstacle_msgs/obs.h"

namespace obstacle_msgs
{

  class MapInfo : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _num_type;
      _num_type num;
      uint32_t obsData_length;
      typedef obstacle_msgs::obs _obsData_type;
      _obsData_type st_obsData;
      _obsData_type * obsData;

    MapInfo():
      header(),
      num(0),
      obsData_length(0), obsData(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->num >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->num >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->num >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->num >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num);
      *(outbuffer + offset + 0) = (this->obsData_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->obsData_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->obsData_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->obsData_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->obsData_length);
      for( uint32_t i = 0; i < obsData_length; i++){
      offset += this->obsData[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->num =  ((uint32_t) (*(inbuffer + offset)));
      this->num |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->num |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->num |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->num);
      uint32_t obsData_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      obsData_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      obsData_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      obsData_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->obsData_length);
      if(obsData_lengthT > obsData_length)
        this->obsData = (obstacle_msgs::obs*)realloc(this->obsData, obsData_lengthT * sizeof(obstacle_msgs::obs));
      obsData_length = obsData_lengthT;
      for( uint32_t i = 0; i < obsData_length; i++){
      offset += this->st_obsData.deserialize(inbuffer + offset);
        memcpy( &(this->obsData[i]), &(this->st_obsData), sizeof(obstacle_msgs::obs));
      }
     return offset;
    }

    const char * getType(){ return "obstacle_msgs/MapInfo"; };
    const char * getMD5(){ return "bb5bd5db7a104c5ecb62217c7b1893ea"; };

  };

}
#endif