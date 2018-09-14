// Generated by gencpp from file universal_teleop/Event.msg
// DO NOT EDIT!


#ifndef UNIVERSAL_TELEOP_MESSAGE_EVENT_H
#define UNIVERSAL_TELEOP_MESSAGE_EVENT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace universal_teleop
{
template <class ContainerAllocator>
struct Event_
{
  typedef Event_<ContainerAllocator> Type;

  Event_()
    : header()
    , event()
    , state(false)  {
    }
  Event_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , event(_alloc)
    , state(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _event_type;
  _event_type event;

   typedef uint8_t _state_type;
  _state_type state;





  typedef boost::shared_ptr< ::universal_teleop::Event_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::universal_teleop::Event_<ContainerAllocator> const> ConstPtr;

}; // struct Event_

typedef ::universal_teleop::Event_<std::allocator<void> > Event;

typedef boost::shared_ptr< ::universal_teleop::Event > EventPtr;
typedef boost::shared_ptr< ::universal_teleop::Event const> EventConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::universal_teleop::Event_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::universal_teleop::Event_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace universal_teleop

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'universal_teleop': ['/home/juan/catkin_ws_2/src/ros-universal-teleop/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::universal_teleop::Event_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::universal_teleop::Event_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::universal_teleop::Event_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::universal_teleop::Event_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::universal_teleop::Event_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::universal_teleop::Event_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::universal_teleop::Event_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7becd39cf7f617d2c6ea4e4c91a2deac";
  }

  static const char* value(const ::universal_teleop::Event_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7becd39cf7f617d2ULL;
  static const uint64_t static_value2 = 0xc6ea4e4c91a2deacULL;
};

template<class ContainerAllocator>
struct DataType< ::universal_teleop::Event_<ContainerAllocator> >
{
  static const char* value()
  {
    return "universal_teleop/Event";
  }

  static const char* value(const ::universal_teleop::Event_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::universal_teleop::Event_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
string event\n\
bool state\n\
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

  static const char* value(const ::universal_teleop::Event_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::universal_teleop::Event_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.event);
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Event_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::universal_teleop::Event_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::universal_teleop::Event_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "event: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.event);
    s << indent << "state: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UNIVERSAL_TELEOP_MESSAGE_EVENT_H
