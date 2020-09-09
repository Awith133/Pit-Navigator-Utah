// Generated by gencpp from file webots_ros/set_floatRequest.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_SET_FLOATREQUEST_H
#define WEBOTS_ROS_MESSAGE_SET_FLOATREQUEST_H


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
struct set_floatRequest_
{
  typedef set_floatRequest_<ContainerAllocator> Type;

  set_floatRequest_()
    : value(0.0)  {
    }
  set_floatRequest_(const ContainerAllocator& _alloc)
    : value(0.0)  {
  (void)_alloc;
    }



   typedef double _value_type;
  _value_type value;





  typedef boost::shared_ptr< ::webots_ros::set_floatRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::set_floatRequest_<ContainerAllocator> const> ConstPtr;

}; // struct set_floatRequest_

typedef ::webots_ros::set_floatRequest_<std::allocator<void> > set_floatRequest;

typedef boost::shared_ptr< ::webots_ros::set_floatRequest > set_floatRequestPtr;
typedef boost::shared_ptr< ::webots_ros::set_floatRequest const> set_floatRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::set_floatRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::set_floatRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace webots_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'webots_ros': ['/home/alex/pit-navigator-utah/catkin_ws/src/Simulation_Control/webots_ros/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::set_floatRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::set_floatRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::set_floatRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::set_floatRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::set_floatRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::set_floatRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::set_floatRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1b1594d2b74931ef8fe7be8e2d594455";
  }

  static const char* value(const ::webots_ros::set_floatRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1b1594d2b74931efULL;
  static const uint64_t static_value2 = 0x8fe7be8e2d594455ULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::set_floatRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/set_floatRequest";
  }

  static const char* value(const ::webots_ros::set_floatRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::set_floatRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 value\n\
";
  }

  static const char* value(const ::webots_ros::set_floatRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::set_floatRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct set_floatRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::webots_ros::set_floatRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::set_floatRequest_<ContainerAllocator>& v)
  {
    s << indent << "value: ";
    Printer<double>::stream(s, indent + "  ", v.value);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_SET_FLOATREQUEST_H
