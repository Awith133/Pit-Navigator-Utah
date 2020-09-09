// Generated by gencpp from file webots_ros/node_get_number_of_contact_points.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_NODE_GET_NUMBER_OF_CONTACT_POINTS_H
#define WEBOTS_ROS_MESSAGE_NODE_GET_NUMBER_OF_CONTACT_POINTS_H

#include <ros/service_traits.h>


#include <webots_ros/node_get_number_of_contact_pointsRequest.h>
#include <webots_ros/node_get_number_of_contact_pointsResponse.h>


namespace webots_ros
{

struct node_get_number_of_contact_points
{

typedef node_get_number_of_contact_pointsRequest Request;
typedef node_get_number_of_contact_pointsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct node_get_number_of_contact_points
} // namespace webots_ros


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::webots_ros::node_get_number_of_contact_points > {
  static const char* value()
  {
    return "b976c7d31b3cf4035732805a489f5d01";
  }

  static const char* value(const ::webots_ros::node_get_number_of_contact_points&) { return value(); }
};

template<>
struct DataType< ::webots_ros::node_get_number_of_contact_points > {
  static const char* value()
  {
    return "webots_ros/node_get_number_of_contact_points";
  }

  static const char* value(const ::webots_ros::node_get_number_of_contact_points&) { return value(); }
};


// service_traits::MD5Sum< ::webots_ros::node_get_number_of_contact_pointsRequest> should match 
// service_traits::MD5Sum< ::webots_ros::node_get_number_of_contact_points > 
template<>
struct MD5Sum< ::webots_ros::node_get_number_of_contact_pointsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_get_number_of_contact_points >::value();
  }
  static const char* value(const ::webots_ros::node_get_number_of_contact_pointsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::webots_ros::node_get_number_of_contact_pointsRequest> should match 
// service_traits::DataType< ::webots_ros::node_get_number_of_contact_points > 
template<>
struct DataType< ::webots_ros::node_get_number_of_contact_pointsRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_get_number_of_contact_points >::value();
  }
  static const char* value(const ::webots_ros::node_get_number_of_contact_pointsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::webots_ros::node_get_number_of_contact_pointsResponse> should match 
// service_traits::MD5Sum< ::webots_ros::node_get_number_of_contact_points > 
template<>
struct MD5Sum< ::webots_ros::node_get_number_of_contact_pointsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_get_number_of_contact_points >::value();
  }
  static const char* value(const ::webots_ros::node_get_number_of_contact_pointsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::webots_ros::node_get_number_of_contact_pointsResponse> should match 
// service_traits::DataType< ::webots_ros::node_get_number_of_contact_points > 
template<>
struct DataType< ::webots_ros::node_get_number_of_contact_pointsResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_get_number_of_contact_points >::value();
  }
  static const char* value(const ::webots_ros::node_get_number_of_contact_pointsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_NODE_GET_NUMBER_OF_CONTACT_POINTS_H
