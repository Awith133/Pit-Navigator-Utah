// Generated by gencpp from file webots_ros/node_get_orientationRequest.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_NODE_GET_ORIENTATIONREQUEST_H
#define WEBOTS_ROS_MESSAGE_NODE_GET_ORIENTATIONREQUEST_H


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
struct node_get_orientationRequest_
{
  typedef node_get_orientationRequest_<ContainerAllocator> Type;

  node_get_orientationRequest_()
    : node(0)  {
    }
  node_get_orientationRequest_(const ContainerAllocator& _alloc)
    : node(0)  {
  (void)_alloc;
    }



   typedef uint64_t _node_type;
  _node_type node;





  typedef boost::shared_ptr< ::webots_ros::node_get_orientationRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::node_get_orientationRequest_<ContainerAllocator> const> ConstPtr;

}; // struct node_get_orientationRequest_

typedef ::webots_ros::node_get_orientationRequest_<std::allocator<void> > node_get_orientationRequest;

typedef boost::shared_ptr< ::webots_ros::node_get_orientationRequest > node_get_orientationRequestPtr;
typedef boost::shared_ptr< ::webots_ros::node_get_orientationRequest const> node_get_orientationRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::node_get_orientationRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::node_get_orientationRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace webots_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'webots_ros': ['/home/alex/pit-navigator-utah/Simulation/catkin_ws/src/old files/webots_ros/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::node_get_orientationRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::node_get_orientationRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::node_get_orientationRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::node_get_orientationRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::node_get_orientationRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::node_get_orientationRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::node_get_orientationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "364f76d0680a5475664d98abd705f036";
  }

  static const char* value(const ::webots_ros::node_get_orientationRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x364f76d0680a5475ULL;
  static const uint64_t static_value2 = 0x664d98abd705f036ULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::node_get_orientationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/node_get_orientationRequest";
  }

  static const char* value(const ::webots_ros::node_get_orientationRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::node_get_orientationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint64 node\n\
";
  }

  static const char* value(const ::webots_ros::node_get_orientationRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::node_get_orientationRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.node);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct node_get_orientationRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::webots_ros::node_get_orientationRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::node_get_orientationRequest_<ContainerAllocator>& v)
  {
    s << indent << "node: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.node);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_NODE_GET_ORIENTATIONREQUEST_H
