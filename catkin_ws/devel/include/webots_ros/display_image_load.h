// Generated by gencpp from file webots_ros/display_image_load.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_DISPLAY_IMAGE_LOAD_H
#define WEBOTS_ROS_MESSAGE_DISPLAY_IMAGE_LOAD_H

#include <ros/service_traits.h>


#include <webots_ros/display_image_loadRequest.h>
#include <webots_ros/display_image_loadResponse.h>


namespace webots_ros
{

struct display_image_load
{

typedef display_image_loadRequest Request;
typedef display_image_loadResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct display_image_load
} // namespace webots_ros


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::webots_ros::display_image_load > {
  static const char* value()
  {
    return "5c71ee48be952f1ade34fd3da67710f3";
  }

  static const char* value(const ::webots_ros::display_image_load&) { return value(); }
};

template<>
struct DataType< ::webots_ros::display_image_load > {
  static const char* value()
  {
    return "webots_ros/display_image_load";
  }

  static const char* value(const ::webots_ros::display_image_load&) { return value(); }
};


// service_traits::MD5Sum< ::webots_ros::display_image_loadRequest> should match 
// service_traits::MD5Sum< ::webots_ros::display_image_load > 
template<>
struct MD5Sum< ::webots_ros::display_image_loadRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::display_image_load >::value();
  }
  static const char* value(const ::webots_ros::display_image_loadRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::webots_ros::display_image_loadRequest> should match 
// service_traits::DataType< ::webots_ros::display_image_load > 
template<>
struct DataType< ::webots_ros::display_image_loadRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::display_image_load >::value();
  }
  static const char* value(const ::webots_ros::display_image_loadRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::webots_ros::display_image_loadResponse> should match 
// service_traits::MD5Sum< ::webots_ros::display_image_load > 
template<>
struct MD5Sum< ::webots_ros::display_image_loadResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::display_image_load >::value();
  }
  static const char* value(const ::webots_ros::display_image_loadResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::webots_ros::display_image_loadResponse> should match 
// service_traits::DataType< ::webots_ros::display_image_load > 
template<>
struct DataType< ::webots_ros::display_image_loadResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::display_image_load >::value();
  }
  static const char* value(const ::webots_ros::display_image_loadResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_DISPLAY_IMAGE_LOAD_H
