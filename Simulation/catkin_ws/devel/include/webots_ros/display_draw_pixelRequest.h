// Generated by gencpp from file webots_ros/display_draw_pixelRequest.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_DISPLAY_DRAW_PIXELREQUEST_H
#define WEBOTS_ROS_MESSAGE_DISPLAY_DRAW_PIXELREQUEST_H


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
struct display_draw_pixelRequest_
{
  typedef display_draw_pixelRequest_<ContainerAllocator> Type;

  display_draw_pixelRequest_()
    : x1(0)
    , y1(0)  {
    }
  display_draw_pixelRequest_(const ContainerAllocator& _alloc)
    : x1(0)
    , y1(0)  {
  (void)_alloc;
    }



   typedef int32_t _x1_type;
  _x1_type x1;

   typedef int32_t _y1_type;
  _y1_type y1;





  typedef boost::shared_ptr< ::webots_ros::display_draw_pixelRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::display_draw_pixelRequest_<ContainerAllocator> const> ConstPtr;

}; // struct display_draw_pixelRequest_

typedef ::webots_ros::display_draw_pixelRequest_<std::allocator<void> > display_draw_pixelRequest;

typedef boost::shared_ptr< ::webots_ros::display_draw_pixelRequest > display_draw_pixelRequestPtr;
typedef boost::shared_ptr< ::webots_ros::display_draw_pixelRequest const> display_draw_pixelRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::display_draw_pixelRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::display_draw_pixelRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::webots_ros::display_draw_pixelRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::display_draw_pixelRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::display_draw_pixelRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::display_draw_pixelRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::display_draw_pixelRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::display_draw_pixelRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::display_draw_pixelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1a417c344be1a26b28c26afdee67552f";
  }

  static const char* value(const ::webots_ros::display_draw_pixelRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1a417c344be1a26bULL;
  static const uint64_t static_value2 = 0x28c26afdee67552fULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::display_draw_pixelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/display_draw_pixelRequest";
  }

  static const char* value(const ::webots_ros::display_draw_pixelRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::display_draw_pixelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 x1\n\
int32 y1\n\
";
  }

  static const char* value(const ::webots_ros::display_draw_pixelRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::display_draw_pixelRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x1);
      stream.next(m.y1);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct display_draw_pixelRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::webots_ros::display_draw_pixelRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::display_draw_pixelRequest_<ContainerAllocator>& v)
  {
    s << indent << "x1: ";
    Printer<int32_t>::stream(s, indent + "  ", v.x1);
    s << indent << "y1: ";
    Printer<int32_t>::stream(s, indent + "  ", v.y1);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_DISPLAY_DRAW_PIXELREQUEST_H
