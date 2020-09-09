// Generated by gencpp from file webots_ros/display_image_pasteRequest.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_DISPLAY_IMAGE_PASTEREQUEST_H
#define WEBOTS_ROS_MESSAGE_DISPLAY_IMAGE_PASTEREQUEST_H


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
struct display_image_pasteRequest_
{
  typedef display_image_pasteRequest_<ContainerAllocator> Type;

  display_image_pasteRequest_()
    : ir(0)
    , x(0)
    , y(0)
    , blend(0)  {
    }
  display_image_pasteRequest_(const ContainerAllocator& _alloc)
    : ir(0)
    , x(0)
    , y(0)
    , blend(0)  {
  (void)_alloc;
    }



   typedef uint64_t _ir_type;
  _ir_type ir;

   typedef int32_t _x_type;
  _x_type x;

   typedef int32_t _y_type;
  _y_type y;

   typedef uint8_t _blend_type;
  _blend_type blend;





  typedef boost::shared_ptr< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> const> ConstPtr;

}; // struct display_image_pasteRequest_

typedef ::webots_ros::display_image_pasteRequest_<std::allocator<void> > display_image_pasteRequest;

typedef boost::shared_ptr< ::webots_ros::display_image_pasteRequest > display_image_pasteRequestPtr;
typedef boost::shared_ptr< ::webots_ros::display_image_pasteRequest const> display_image_pasteRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::display_image_pasteRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0f2c4510b03c29183c89e4cfbb8df79e";
  }

  static const char* value(const ::webots_ros::display_image_pasteRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0f2c4510b03c2918ULL;
  static const uint64_t static_value2 = 0x3c89e4cfbb8df79eULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/display_image_pasteRequest";
  }

  static const char* value(const ::webots_ros::display_image_pasteRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint64 ir\n\
int32 x\n\
int32 y\n\
uint8 blend\n\
";
  }

  static const char* value(const ::webots_ros::display_image_pasteRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ir);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.blend);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct display_image_pasteRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::display_image_pasteRequest_<ContainerAllocator>& v)
  {
    s << indent << "ir: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.ir);
    s << indent << "x: ";
    Printer<int32_t>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<int32_t>::stream(s, indent + "  ", v.y);
    s << indent << "blend: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.blend);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_DISPLAY_IMAGE_PASTEREQUEST_H
