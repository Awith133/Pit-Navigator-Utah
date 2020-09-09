// Generated by gencpp from file webots_ros/field_get_int32Request.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_FIELD_GET_INT32REQUEST_H
#define WEBOTS_ROS_MESSAGE_FIELD_GET_INT32REQUEST_H


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
struct field_get_int32Request_
{
  typedef field_get_int32Request_<ContainerAllocator> Type;

  field_get_int32Request_()
    : field(0)
    , index(0)  {
    }
  field_get_int32Request_(const ContainerAllocator& _alloc)
    : field(0)
    , index(0)  {
  (void)_alloc;
    }



   typedef uint64_t _field_type;
  _field_type field;

   typedef int32_t _index_type;
  _index_type index;





  typedef boost::shared_ptr< ::webots_ros::field_get_int32Request_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::field_get_int32Request_<ContainerAllocator> const> ConstPtr;

}; // struct field_get_int32Request_

typedef ::webots_ros::field_get_int32Request_<std::allocator<void> > field_get_int32Request;

typedef boost::shared_ptr< ::webots_ros::field_get_int32Request > field_get_int32RequestPtr;
typedef boost::shared_ptr< ::webots_ros::field_get_int32Request const> field_get_int32RequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::field_get_int32Request_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::field_get_int32Request_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::webots_ros::field_get_int32Request_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::field_get_int32Request_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::field_get_int32Request_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::field_get_int32Request_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::field_get_int32Request_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::field_get_int32Request_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::field_get_int32Request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "828947932ebb9e813d3ef918f20f0d81";
  }

  static const char* value(const ::webots_ros::field_get_int32Request_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x828947932ebb9e81ULL;
  static const uint64_t static_value2 = 0x3d3ef918f20f0d81ULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::field_get_int32Request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/field_get_int32Request";
  }

  static const char* value(const ::webots_ros::field_get_int32Request_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::field_get_int32Request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint64 field\n\
int32 index\n\
";
  }

  static const char* value(const ::webots_ros::field_get_int32Request_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::field_get_int32Request_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.field);
      stream.next(m.index);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct field_get_int32Request_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::webots_ros::field_get_int32Request_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::field_get_int32Request_<ContainerAllocator>& v)
  {
    s << indent << "field: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.field);
    s << indent << "index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.index);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_FIELD_GET_INT32REQUEST_H
