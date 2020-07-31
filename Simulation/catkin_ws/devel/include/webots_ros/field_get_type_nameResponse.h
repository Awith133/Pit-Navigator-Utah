// Generated by gencpp from file webots_ros/field_get_type_nameResponse.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_FIELD_GET_TYPE_NAMERESPONSE_H
#define WEBOTS_ROS_MESSAGE_FIELD_GET_TYPE_NAMERESPONSE_H


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
struct field_get_type_nameResponse_
{
  typedef field_get_type_nameResponse_<ContainerAllocator> Type;

  field_get_type_nameResponse_()
    : name()  {
    }
  field_get_type_nameResponse_(const ContainerAllocator& _alloc)
    : name(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;





  typedef boost::shared_ptr< ::webots_ros::field_get_type_nameResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::field_get_type_nameResponse_<ContainerAllocator> const> ConstPtr;

}; // struct field_get_type_nameResponse_

typedef ::webots_ros::field_get_type_nameResponse_<std::allocator<void> > field_get_type_nameResponse;

typedef boost::shared_ptr< ::webots_ros::field_get_type_nameResponse > field_get_type_nameResponsePtr;
typedef boost::shared_ptr< ::webots_ros::field_get_type_nameResponse const> field_get_type_nameResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::field_get_type_nameResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::field_get_type_nameResponse_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::webots_ros::field_get_type_nameResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::field_get_type_nameResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::field_get_type_nameResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::field_get_type_nameResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::field_get_type_nameResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::field_get_type_nameResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::field_get_type_nameResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c1f3d28f1b044c871e6eff2e9fc3c667";
  }

  static const char* value(const ::webots_ros::field_get_type_nameResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc1f3d28f1b044c87ULL;
  static const uint64_t static_value2 = 0x1e6eff2e9fc3c667ULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::field_get_type_nameResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/field_get_type_nameResponse";
  }

  static const char* value(const ::webots_ros::field_get_type_nameResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::field_get_type_nameResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string name\n\
\n\
";
  }

  static const char* value(const ::webots_ros::field_get_type_nameResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::field_get_type_nameResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct field_get_type_nameResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::webots_ros::field_get_type_nameResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::field_get_type_nameResponse_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_FIELD_GET_TYPE_NAMERESPONSE_H
