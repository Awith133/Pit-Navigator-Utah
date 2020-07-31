// Generated by gencpp from file webots_ros/gps_decimal_degrees_to_degrees_minutes_secondsResponse.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_GPS_DECIMAL_DEGREES_TO_DEGREES_MINUTES_SECONDSRESPONSE_H
#define WEBOTS_ROS_MESSAGE_GPS_DECIMAL_DEGREES_TO_DEGREES_MINUTES_SECONDSRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace webots_ros
{
template <class ContainerAllocator>
struct gps_decimal_degrees_to_degrees_minutes_secondsResponse_
{
  typedef gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> Type;

  gps_decimal_degrees_to_degrees_minutes_secondsResponse_()
    : degreesMinutesSeconds()  {
    }
  gps_decimal_degrees_to_degrees_minutes_secondsResponse_(const ContainerAllocator& _alloc)
    : degreesMinutesSeconds(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _degreesMinutesSeconds_type;
  _degreesMinutesSeconds_type degreesMinutesSeconds;





  typedef boost::shared_ptr< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct gps_decimal_degrees_to_degrees_minutes_secondsResponse_

typedef ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<std::allocator<void> > gps_decimal_degrees_to_degrees_minutes_secondsResponse;

typedef boost::shared_ptr< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse > gps_decimal_degrees_to_degrees_minutes_secondsResponsePtr;
typedef boost::shared_ptr< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse const> gps_decimal_degrees_to_degrees_minutes_secondsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace webots_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'webots_ros': ['/home/alex/pit-navigator-utah/Simulation/catkin_ws/src/old files/webots_ros/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1dd167c46cb823612e4149b12fa41046";
  }

  static const char* value(const ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1dd167c46cb82361ULL;
  static const uint64_t static_value2 = 0x2e4149b12fa41046ULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/gps_decimal_degrees_to_degrees_minutes_secondsResponse";
  }

  static const char* value(const ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string degreesMinutesSeconds\n\
\n\
";
  }

  static const char* value(const ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.degreesMinutesSeconds);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct gps_decimal_degrees_to_degrees_minutes_secondsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator>& v)
  {
    s << indent << "degreesMinutesSeconds: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.degreesMinutesSeconds);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_GPS_DECIMAL_DEGREES_TO_DEGREES_MINUTES_SECONDSRESPONSE_H
