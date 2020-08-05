// Generated by gencpp from file webots_ros/speaker_is_sound_playingRequest.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_SPEAKER_IS_SOUND_PLAYINGREQUEST_H
#define WEBOTS_ROS_MESSAGE_SPEAKER_IS_SOUND_PLAYINGREQUEST_H


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
struct speaker_is_sound_playingRequest_
{
  typedef speaker_is_sound_playingRequest_<ContainerAllocator> Type;

  speaker_is_sound_playingRequest_()
    : sound()  {
    }
  speaker_is_sound_playingRequest_(const ContainerAllocator& _alloc)
    : sound(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _sound_type;
  _sound_type sound;





  typedef boost::shared_ptr< ::webots_ros::speaker_is_sound_playingRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::speaker_is_sound_playingRequest_<ContainerAllocator> const> ConstPtr;

}; // struct speaker_is_sound_playingRequest_

typedef ::webots_ros::speaker_is_sound_playingRequest_<std::allocator<void> > speaker_is_sound_playingRequest;

typedef boost::shared_ptr< ::webots_ros::speaker_is_sound_playingRequest > speaker_is_sound_playingRequestPtr;
typedef boost::shared_ptr< ::webots_ros::speaker_is_sound_playingRequest const> speaker_is_sound_playingRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::speaker_is_sound_playingRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::speaker_is_sound_playingRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace webots_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'webots_ros': ['/home/alex/pit-navigator-utah/Simulation/catkin_ws/src/webots_ros/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::speaker_is_sound_playingRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::speaker_is_sound_playingRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::speaker_is_sound_playingRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::speaker_is_sound_playingRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::speaker_is_sound_playingRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::speaker_is_sound_playingRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::speaker_is_sound_playingRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6f8594d24e685f0ca0a97807507c3483";
  }

  static const char* value(const ::webots_ros::speaker_is_sound_playingRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6f8594d24e685f0cULL;
  static const uint64_t static_value2 = 0xa0a97807507c3483ULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::speaker_is_sound_playingRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/speaker_is_sound_playingRequest";
  }

  static const char* value(const ::webots_ros::speaker_is_sound_playingRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::speaker_is_sound_playingRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string sound\n\
";
  }

  static const char* value(const ::webots_ros::speaker_is_sound_playingRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::speaker_is_sound_playingRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.sound);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct speaker_is_sound_playingRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::webots_ros::speaker_is_sound_playingRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::speaker_is_sound_playingRequest_<ContainerAllocator>& v)
  {
    s << indent << "sound: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.sound);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_SPEAKER_IS_SOUND_PLAYINGREQUEST_H
