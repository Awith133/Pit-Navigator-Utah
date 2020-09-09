// Generated by gencpp from file webots_ros/BoolStamped.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_BOOLSTAMPED_H
#define WEBOTS_ROS_MESSAGE_BOOLSTAMPED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace webots_ros
{
template <class ContainerAllocator>
struct BoolStamped_
{
  typedef BoolStamped_<ContainerAllocator> Type;

  BoolStamped_()
    : header()
    , data(false)  {
    }
  BoolStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , data(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::webots_ros::BoolStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::BoolStamped_<ContainerAllocator> const> ConstPtr;

}; // struct BoolStamped_

typedef ::webots_ros::BoolStamped_<std::allocator<void> > BoolStamped;

typedef boost::shared_ptr< ::webots_ros::BoolStamped > BoolStampedPtr;
typedef boost::shared_ptr< ::webots_ros::BoolStamped const> BoolStampedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::BoolStamped_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::BoolStamped_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace webots_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'webots_ros': ['/home/alex/pit-navigator-utah/catkin_ws/src/Simulation_Control/webots_ros/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::BoolStamped_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::BoolStamped_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::BoolStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::BoolStamped_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::BoolStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::BoolStamped_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::BoolStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "542e22b190dc8e6eb476d50dda88feb7";
  }

  static const char* value(const ::webots_ros::BoolStamped_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x542e22b190dc8e6eULL;
  static const uint64_t static_value2 = 0xb476d50dda88feb7ULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::BoolStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/BoolStamped";
  }

  static const char* value(const ::webots_ros::BoolStamped_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::BoolStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
bool data\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::webots_ros::BoolStamped_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::BoolStamped_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BoolStamped_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::webots_ros::BoolStamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::BoolStamped_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "data: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_BOOLSTAMPED_H
