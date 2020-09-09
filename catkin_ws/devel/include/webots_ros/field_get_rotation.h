// Generated by gencpp from file webots_ros/field_get_rotation.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_FIELD_GET_ROTATION_H
#define WEBOTS_ROS_MESSAGE_FIELD_GET_ROTATION_H

#include <ros/service_traits.h>


#include <webots_ros/field_get_rotationRequest.h>
#include <webots_ros/field_get_rotationResponse.h>


namespace webots_ros
{

struct field_get_rotation
{

typedef field_get_rotationRequest Request;
typedef field_get_rotationResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct field_get_rotation
} // namespace webots_ros


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::webots_ros::field_get_rotation > {
  static const char* value()
  {
    return "ad330bd22eabd1d11d7ec7469e804a11";
  }

  static const char* value(const ::webots_ros::field_get_rotation&) { return value(); }
};

template<>
struct DataType< ::webots_ros::field_get_rotation > {
  static const char* value()
  {
    return "webots_ros/field_get_rotation";
  }

  static const char* value(const ::webots_ros::field_get_rotation&) { return value(); }
};


// service_traits::MD5Sum< ::webots_ros::field_get_rotationRequest> should match 
// service_traits::MD5Sum< ::webots_ros::field_get_rotation > 
template<>
struct MD5Sum< ::webots_ros::field_get_rotationRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::field_get_rotation >::value();
  }
  static const char* value(const ::webots_ros::field_get_rotationRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::webots_ros::field_get_rotationRequest> should match 
// service_traits::DataType< ::webots_ros::field_get_rotation > 
template<>
struct DataType< ::webots_ros::field_get_rotationRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::field_get_rotation >::value();
  }
  static const char* value(const ::webots_ros::field_get_rotationRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::webots_ros::field_get_rotationResponse> should match 
// service_traits::MD5Sum< ::webots_ros::field_get_rotation > 
template<>
struct MD5Sum< ::webots_ros::field_get_rotationResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::field_get_rotation >::value();
  }
  static const char* value(const ::webots_ros::field_get_rotationResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::webots_ros::field_get_rotationResponse> should match 
// service_traits::DataType< ::webots_ros::field_get_rotation > 
template<>
struct DataType< ::webots_ros::field_get_rotationResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::field_get_rotation >::value();
  }
  static const char* value(const ::webots_ros::field_get_rotationResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_FIELD_GET_ROTATION_H
