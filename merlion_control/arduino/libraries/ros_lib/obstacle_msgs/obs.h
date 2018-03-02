#ifndef _ROS_obstacle_msgs_obs_h
#define _ROS_obstacle_msgs_obs_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "obstacle_msgs/point3.h"

namespace obstacle_msgs
{

  class obs : public ros::Msg
  {
    public:
      typedef uint64_t _identityID_type;
      _identityID_type identityID;
      typedef obstacle_msgs::point3 _centerPos_type;
      _centerPos_type centerPos;
      typedef float _diameter_type;
      _diameter_type diameter;
      typedef float _height_type;
      _height_type height;
      uint32_t histogram_length;
      typedef float _histogram_type;
      _histogram_type st_histogram;
      _histogram_type * histogram;
      typedef const char* _classes_type;
      _classes_type classes;
      typedef float _probability_type;
      _probability_type probability;

    obs():
      identityID(0),
      centerPos(),
      diameter(0),
      height(0),
      histogram_length(0), histogram(NULL),
      classes(""),
      probability(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        uint64_t real;
        uint32_t base;
      } u_identityID;
      u_identityID.real = this->identityID;
      *(outbuffer + offset + 0) = (u_identityID.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_identityID.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_identityID.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_identityID.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->identityID);
      offset += this->centerPos.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_diameter;
      u_diameter.real = this->diameter;
      *(outbuffer + offset + 0) = (u_diameter.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_diameter.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_diameter.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_diameter.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->diameter);
      union {
        float real;
        uint32_t base;
      } u_height;
      u_height.real = this->height;
      *(outbuffer + offset + 0) = (u_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      *(outbuffer + offset + 0) = (this->histogram_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->histogram_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->histogram_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->histogram_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->histogram_length);
      for( uint32_t i = 0; i < histogram_length; i++){
      union {
        float real;
        uint32_t base;
      } u_histogrami;
      u_histogrami.real = this->histogram[i];
      *(outbuffer + offset + 0) = (u_histogrami.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_histogrami.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_histogrami.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_histogrami.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->histogram[i]);
      }
      uint32_t length_classes = strlen(this->classes);
      varToArr(outbuffer + offset, length_classes);
      offset += 4;
      memcpy(outbuffer + offset, this->classes, length_classes);
      offset += length_classes;
      union {
        float real;
        uint32_t base;
      } u_probability;
      u_probability.real = this->probability;
      *(outbuffer + offset + 0) = (u_probability.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_probability.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_probability.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_probability.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->probability);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        uint64_t real;
        uint32_t base;
      } u_identityID;
      u_identityID.base = 0;
      u_identityID.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_identityID.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_identityID.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_identityID.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->identityID = u_identityID.real;
      offset += sizeof(this->identityID);
      offset += this->centerPos.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_diameter;
      u_diameter.base = 0;
      u_diameter.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_diameter.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_diameter.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_diameter.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->diameter = u_diameter.real;
      offset += sizeof(this->diameter);
      union {
        float real;
        uint32_t base;
      } u_height;
      u_height.base = 0;
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->height = u_height.real;
      offset += sizeof(this->height);
      uint32_t histogram_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      histogram_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      histogram_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      histogram_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->histogram_length);
      if(histogram_lengthT > histogram_length)
        this->histogram = (float*)realloc(this->histogram, histogram_lengthT * sizeof(float));
      histogram_length = histogram_lengthT;
      for( uint32_t i = 0; i < histogram_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_histogram;
      u_st_histogram.base = 0;
      u_st_histogram.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_histogram.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_histogram.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_histogram.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_histogram = u_st_histogram.real;
      offset += sizeof(this->st_histogram);
        memcpy( &(this->histogram[i]), &(this->st_histogram), sizeof(float));
      }
      uint32_t length_classes;
      arrToVar(length_classes, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_classes; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_classes-1]=0;
      this->classes = (char *)(inbuffer + offset-1);
      offset += length_classes;
      union {
        float real;
        uint32_t base;
      } u_probability;
      u_probability.base = 0;
      u_probability.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_probability.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_probability.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_probability.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->probability = u_probability.real;
      offset += sizeof(this->probability);
     return offset;
    }

    const char * getType(){ return "obstacle_msgs/obs"; };
    const char * getMD5(){ return "03f7a24f34510141fe14af6dad16ac51"; };

  };

}
#endif