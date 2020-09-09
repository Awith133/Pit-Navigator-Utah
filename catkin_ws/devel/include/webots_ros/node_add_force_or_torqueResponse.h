// Generated by gencpp from file webots_ros/node_add_force_or_torqueResponse.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_NODE_ADD_FORCE_OR_TORQUERESPONSE_H
#define WEBOTS_ROS_MESSAGE_NODE_ADD_FORCE_OR_TORQUERESPONSE_H


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
struct node_add_force_or_torqueResponse_
{
  typedef node_add_force_or_torqueResponse_<ContainerAllocator> Type;

  node_add_force_or_torqueResponse_()
    : success(0)  {
    }
  node_add_force_or_torqueResponse_(const ContainerAllocator& _alloc)
    : success(0)  {
  (void)_alloc;
    }



   typedef int32_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::webots_ros::node_add_force_or_torqueResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::node_add_force_or_torqueResponse_<ContainerAllocator> const> ConstPtr;

}; // struct node_add_force_or_torqueResponse_

typedef ::webots_ros::node_add_force_or_torqueResponse_<std::allocator<void> > node_add_force_or_torqueResponse;

typedef boost::shared_ptr< ::webots_ros::node_add_force_or_torqueResponse > node_add_force_or_torqueResponsePtr;
typedef boost::shared_ptr< ::webots_ros::node_add_force_or_torqueResponse const> node_add_force_or_torqueResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::node_add_force_or_torqueResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::node_add_force_or_torqueResponse_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::webots_ros::node_add_force_or_torqueResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::node_add_force_or_torqueResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::node_add_force_or_torqueResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::node_add_force_or_torqueResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::node_add_force_or_torqueResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::node_add_force_or_torqueResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::node_add_force_or_torqueResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3c2bcf2ff0894cb3058b1bf4c8c4175a";
  }

  static const char* value(const ::webots_ros::node_add_force_or_torqueResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3c2bcf2ff0894cb3ULL;
  static const uint64_t static_value2 = 0x058b1bf4c8c4175aULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::node_add_force_or_torqueResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/node_add_force_or_torqueResponse";
  }

  static const char* value(const ::webots_ros::node_add_force_or_torqueResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::node_add_force_or_torqueResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 success\n\
\n\
";
  }

  static const char* value(const ::webots_ros::node_add_force_or_torqueResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::node_add_force_or_torqueResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct node_add_force_or_torqueResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::webots_ros::node_add_force_or_torqueResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::node_add_force_or_torqueResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<int32_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_NODE_ADD_FORCE_OR_TORQUERESPONSE_H
