// Generated by gencpp from file webots_ros/get_float.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_GET_FLOAT_H
#define WEBOTS_ROS_MESSAGE_GET_FLOAT_H

#include <ros/service_traits.h>


#include <webots_ros/get_floatRequest.h>
#include <webots_ros/get_floatResponse.h>


namespace webots_ros
{

struct get_float
{

typedef get_floatRequest Request;
typedef get_floatResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct get_float
} // namespace webots_ros


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::webots_ros::get_float > {
  static const char* value()
  {
    return "896e758cc9f93e6fc99057486a02ac6a";
  }

  static const char* value(const ::webots_ros::get_float&) { return value(); }
};

template<>
struct DataType< ::webots_ros::get_float > {
  static const char* value()
  {
    return "webots_ros/get_float";
  }

  static const char* value(const ::webots_ros::get_float&) { return value(); }
};


// service_traits::MD5Sum< ::webots_ros::get_floatRequest> should match 
// service_traits::MD5Sum< ::webots_ros::get_float > 
template<>
struct MD5Sum< ::webots_ros::get_floatRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::get_float >::value();
  }
  static const char* value(const ::webots_ros::get_floatRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::webots_ros::get_floatRequest> should match 
// service_traits::DataType< ::webots_ros::get_float > 
template<>
struct DataType< ::webots_ros::get_floatRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::get_float >::value();
  }
  static const char* value(const ::webots_ros::get_floatRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::webots_ros::get_floatResponse> should match 
// service_traits::MD5Sum< ::webots_ros::get_float > 
template<>
struct MD5Sum< ::webots_ros::get_floatResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::get_float >::value();
  }
  static const char* value(const ::webots_ros::get_floatResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::webots_ros::get_floatResponse> should match 
// service_traits::DataType< ::webots_ros::get_float > 
template<>
struct DataType< ::webots_ros::get_floatResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::get_float >::value();
  }
  static const char* value(const ::webots_ros::get_floatResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_GET_FLOAT_H
