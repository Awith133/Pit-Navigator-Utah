// Generated by gencpp from file webots_ros/robot_get_device_list.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_ROBOT_GET_DEVICE_LIST_H
#define WEBOTS_ROS_MESSAGE_ROBOT_GET_DEVICE_LIST_H

#include <ros/service_traits.h>


#include <webots_ros/robot_get_device_listRequest.h>
#include <webots_ros/robot_get_device_listResponse.h>


namespace webots_ros
{

struct robot_get_device_list
{

typedef robot_get_device_listRequest Request;
typedef robot_get_device_listResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct robot_get_device_list
} // namespace webots_ros


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::webots_ros::robot_get_device_list > {
  static const char* value()
  {
    return "463f6db5695c1f090f76a453a11e7381";
  }

  static const char* value(const ::webots_ros::robot_get_device_list&) { return value(); }
};

template<>
struct DataType< ::webots_ros::robot_get_device_list > {
  static const char* value()
  {
    return "webots_ros/robot_get_device_list";
  }

  static const char* value(const ::webots_ros::robot_get_device_list&) { return value(); }
};


// service_traits::MD5Sum< ::webots_ros::robot_get_device_listRequest> should match 
// service_traits::MD5Sum< ::webots_ros::robot_get_device_list > 
template<>
struct MD5Sum< ::webots_ros::robot_get_device_listRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::robot_get_device_list >::value();
  }
  static const char* value(const ::webots_ros::robot_get_device_listRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::webots_ros::robot_get_device_listRequest> should match 
// service_traits::DataType< ::webots_ros::robot_get_device_list > 
template<>
struct DataType< ::webots_ros::robot_get_device_listRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::robot_get_device_list >::value();
  }
  static const char* value(const ::webots_ros::robot_get_device_listRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::webots_ros::robot_get_device_listResponse> should match 
// service_traits::MD5Sum< ::webots_ros::robot_get_device_list > 
template<>
struct MD5Sum< ::webots_ros::robot_get_device_listResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::robot_get_device_list >::value();
  }
  static const char* value(const ::webots_ros::robot_get_device_listResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::webots_ros::robot_get_device_listResponse> should match 
// service_traits::DataType< ::webots_ros::robot_get_device_list > 
template<>
struct DataType< ::webots_ros::robot_get_device_listResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::robot_get_device_list >::value();
  }
  static const char* value(const ::webots_ros::robot_get_device_listResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_ROBOT_GET_DEVICE_LIST_H
