#ifndef EGOCYLINDRICAL_SENSOR_FILTER_H
#define EGOCYLINDRICAL_SENSOR_FILTER_H

///NOTE: Having sensor.h include this file ensures that this functionality is available by including just sensor.h while also avoiding a circular dependency
//#include <egocylindrical/sensor.h>  
// #include <ros/message_traits.h>


namespace message_filters
{
namespace message_traits
{
  template <>
  struct IsMessage< ::egocylindrical::utils::SensorMeasurement >
    : TrueType
    { };
    
  template <>
  struct HasHeader< ::egocylindrical::utils::SensorMeasurement >
    : TrueType
    { };

}
}

#endif  //EGOCYLINDRICAL_SENSOR_FILTER_H
