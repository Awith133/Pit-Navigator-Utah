// Generated by gencpp from file webots_ros/supervisor_movie_start_recording.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_SUPERVISOR_MOVIE_START_RECORDING_H
#define WEBOTS_ROS_MESSAGE_SUPERVISOR_MOVIE_START_RECORDING_H

#include <ros/service_traits.h>


#include <webots_ros/supervisor_movie_start_recordingRequest.h>
#include <webots_ros/supervisor_movie_start_recordingResponse.h>


namespace webots_ros
{

struct supervisor_movie_start_recording
{

typedef supervisor_movie_start_recordingRequest Request;
typedef supervisor_movie_start_recordingResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct supervisor_movie_start_recording
} // namespace webots_ros


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::webots_ros::supervisor_movie_start_recording > {
  static const char* value()
  {
    return "96ca298eece1e7a6fe756c404839bdcc";
  }

  static const char* value(const ::webots_ros::supervisor_movie_start_recording&) { return value(); }
};

template<>
struct DataType< ::webots_ros::supervisor_movie_start_recording > {
  static const char* value()
  {
    return "webots_ros/supervisor_movie_start_recording";
  }

  static const char* value(const ::webots_ros::supervisor_movie_start_recording&) { return value(); }
};


// service_traits::MD5Sum< ::webots_ros::supervisor_movie_start_recordingRequest> should match 
// service_traits::MD5Sum< ::webots_ros::supervisor_movie_start_recording > 
template<>
struct MD5Sum< ::webots_ros::supervisor_movie_start_recordingRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::supervisor_movie_start_recording >::value();
  }
  static const char* value(const ::webots_ros::supervisor_movie_start_recordingRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::webots_ros::supervisor_movie_start_recordingRequest> should match 
// service_traits::DataType< ::webots_ros::supervisor_movie_start_recording > 
template<>
struct DataType< ::webots_ros::supervisor_movie_start_recordingRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::supervisor_movie_start_recording >::value();
  }
  static const char* value(const ::webots_ros::supervisor_movie_start_recordingRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::webots_ros::supervisor_movie_start_recordingResponse> should match 
// service_traits::MD5Sum< ::webots_ros::supervisor_movie_start_recording > 
template<>
struct MD5Sum< ::webots_ros::supervisor_movie_start_recordingResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::supervisor_movie_start_recording >::value();
  }
  static const char* value(const ::webots_ros::supervisor_movie_start_recordingResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::webots_ros::supervisor_movie_start_recordingResponse> should match 
// service_traits::DataType< ::webots_ros::supervisor_movie_start_recording > 
template<>
struct DataType< ::webots_ros::supervisor_movie_start_recordingResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::supervisor_movie_start_recording >::value();
  }
  static const char* value(const ::webots_ros::supervisor_movie_start_recordingResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_SUPERVISOR_MOVIE_START_RECORDING_H
