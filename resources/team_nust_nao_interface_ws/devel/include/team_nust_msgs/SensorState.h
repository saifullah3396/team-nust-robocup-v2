// Generated by gencpp from file team_nust_msgs/SensorState.msg
// DO NOT EDIT!


#ifndef TEAM_NUST_MSGS_MESSAGE_SENSORSTATE_H
#define TEAM_NUST_MSGS_MESSAGE_SENSORSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace team_nust_msgs
{
template <class ContainerAllocator>
struct SensorState_
{
  typedef SensorState_<ContainerAllocator> Type;

  SensorState_()
    : header()
    , name()
    , value()  {
    }
  SensorState_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , name(_alloc)
    , value(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _name_type;
  _name_type name;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _value_type;
  _value_type value;




  typedef boost::shared_ptr< ::team_nust_msgs::SensorState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::team_nust_msgs::SensorState_<ContainerAllocator> const> ConstPtr;

}; // struct SensorState_

typedef ::team_nust_msgs::SensorState_<std::allocator<void> > SensorState;

typedef boost::shared_ptr< ::team_nust_msgs::SensorState > SensorStatePtr;
typedef boost::shared_ptr< ::team_nust_msgs::SensorState const> SensorStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::team_nust_msgs::SensorState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::team_nust_msgs::SensorState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace team_nust_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'team_nust_msgs': ['/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'humanoid_nav_msgs': ['/opt/ros/indigo/share/humanoid_nav_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::team_nust_msgs::SensorState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::team_nust_msgs::SensorState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::team_nust_msgs::SensorState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::team_nust_msgs::SensorState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::team_nust_msgs::SensorState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::team_nust_msgs::SensorState_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::team_nust_msgs::SensorState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "44860e07301dfb37702ea8848351bb12";
  }

  static const char* value(const ::team_nust_msgs::SensorState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x44860e07301dfb37ULL;
  static const uint64_t static_value2 = 0x702ea8848351bb12ULL;
};

template<class ContainerAllocator>
struct DataType< ::team_nust_msgs::SensorState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "team_nust_msgs/SensorState";
  }

  static const char* value(const ::team_nust_msgs::SensorState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::team_nust_msgs::SensorState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
string[] name\n\
float64[] value\n\
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

  static const char* value(const ::team_nust_msgs::SensorState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::team_nust_msgs::SensorState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.name);
      stream.next(m.value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SensorState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::team_nust_msgs::SensorState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::team_nust_msgs::SensorState_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "name[]" << std::endl;
    for (size_t i = 0; i < v.name.size(); ++i)
    {
      s << indent << "  name[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name[i]);
    }
    s << indent << "value[]" << std::endl;
    for (size_t i = 0; i < v.value.size(); ++i)
    {
      s << indent << "  value[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.value[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // TEAM_NUST_MSGS_MESSAGE_SENSORSTATE_H
