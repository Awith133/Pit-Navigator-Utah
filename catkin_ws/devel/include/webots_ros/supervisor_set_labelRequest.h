// Generated by gencpp from file webots_ros/supervisor_set_labelRequest.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_SUPERVISOR_SET_LABELREQUEST_H
#define WEBOTS_ROS_MESSAGE_SUPERVISOR_SET_LABELREQUEST_H


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
struct supervisor_set_labelRequest_
{
  typedef supervisor_set_labelRequest_<ContainerAllocator> Type;

  supervisor_set_labelRequest_()
    : id(0)
    , label()
    , xpos(0.0)
    , ypos(0.0)
    , size(0.0)
    , color(0)
    , transparency(0.0)
    , font()  {
    }
  supervisor_set_labelRequest_(const ContainerAllocator& _alloc)
    : id(0)
    , label(_alloc)
    , xpos(0.0)
    , ypos(0.0)
    , size(0.0)
    , color(0)
    , transparency(0.0)
    , font(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _id_type;
  _id_type id;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _label_type;
  _label_type label;

   typedef double _xpos_type;
  _xpos_type xpos;

   typedef double _ypos_type;
  _ypos_type ypos;

   typedef double _size_type;
  _size_type size;

   typedef int32_t _color_type;
  _color_type color;

   typedef double _transparency_type;
  _transparency_type transparency;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _font_type;
  _font_type font;





  typedef boost::shared_ptr< ::webots_ros::supervisor_set_labelRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::supervisor_set_labelRequest_<ContainerAllocator> const> ConstPtr;

}; // struct supervisor_set_labelRequest_

typedef ::webots_ros::supervisor_set_labelRequest_<std::allocator<void> > supervisor_set_labelRequest;

typedef boost::shared_ptr< ::webots_ros::supervisor_set_labelRequest > supervisor_set_labelRequestPtr;
typedef boost::shared_ptr< ::webots_ros::supervisor_set_labelRequest const> supervisor_set_labelRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::supervisor_set_labelRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::supervisor_set_labelRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace webots_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'webots_ros': ['/home/alex/pit-navigator-utah/catkin_ws/src/Simulation_Control/webots_ros/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::supervisor_set_labelRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::supervisor_set_labelRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::supervisor_set_labelRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::supervisor_set_labelRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::supervisor_set_labelRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::supervisor_set_labelRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::supervisor_set_labelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "383d62b53c2b62c441c9996504515955";
  }

  static const char* value(const ::webots_ros::supervisor_set_labelRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x383d62b53c2b62c4ULL;
  static const uint64_t static_value2 = 0x41c9996504515955ULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::supervisor_set_labelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/supervisor_set_labelRequest";
  }

  static const char* value(const ::webots_ros::supervisor_set_labelRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::supervisor_set_labelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 id\n\
string label\n\
float64 xpos\n\
float64 ypos\n\
float64 size\n\
int32 color\n\
float64 transparency\n\
string font\n\
";
  }

  static const char* value(const ::webots_ros::supervisor_set_labelRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::supervisor_set_labelRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.label);
      stream.next(m.xpos);
      stream.next(m.ypos);
      stream.next(m.size);
      stream.next(m.color);
      stream.next(m.transparency);
      stream.next(m.font);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct supervisor_set_labelRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::webots_ros::supervisor_set_labelRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::supervisor_set_labelRequest_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "label: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.label);
    s << indent << "xpos: ";
    Printer<double>::stream(s, indent + "  ", v.xpos);
    s << indent << "ypos: ";
    Printer<double>::stream(s, indent + "  ", v.ypos);
    s << indent << "size: ";
    Printer<double>::stream(s, indent + "  ", v.size);
    s << indent << "color: ";
    Printer<int32_t>::stream(s, indent + "  ", v.color);
    s << indent << "transparency: ";
    Printer<double>::stream(s, indent + "  ", v.transparency);
    s << indent << "font: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.font);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_SUPERVISOR_SET_LABELREQUEST_H
