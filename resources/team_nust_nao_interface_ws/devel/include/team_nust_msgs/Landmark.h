// Generated by gencpp from file team_nust_msgs/Landmark.msg
// DO NOT EDIT!


#ifndef TEAM_NUST_MSGS_MESSAGE_LANDMARK_H
#define TEAM_NUST_MSGS_MESSAGE_LANDMARK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace team_nust_msgs
{
template <class ContainerAllocator>
struct Landmark_
{
  typedef Landmark_<ContainerAllocator> Type;

  Landmark_()
    : type(0)
    , pos()  {
    }
  Landmark_(const ContainerAllocator& _alloc)
    : type(0)
    , pos(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _type_type;
  _type_type type;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _pos_type;
  _pos_type pos;




  typedef boost::shared_ptr< ::team_nust_msgs::Landmark_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::team_nust_msgs::Landmark_<ContainerAllocator> const> ConstPtr;

}; // struct Landmark_

typedef ::team_nust_msgs::Landmark_<std::allocator<void> > Landmark;

typedef boost::shared_ptr< ::team_nust_msgs::Landmark > LandmarkPtr;
typedef boost::shared_ptr< ::team_nust_msgs::Landmark const> LandmarkConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::team_nust_msgs::Landmark_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::team_nust_msgs::Landmark_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace team_nust_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'team_nust_msgs': ['/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'humanoid_nav_msgs': ['/opt/ros/indigo/share/humanoid_nav_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::team_nust_msgs::Landmark_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::team_nust_msgs::Landmark_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::team_nust_msgs::Landmark_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::team_nust_msgs::Landmark_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::team_nust_msgs::Landmark_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::team_nust_msgs::Landmark_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::team_nust_msgs::Landmark_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a44c2e55d48971b074b6d4f211c997d7";
  }

  static const char* value(const ::team_nust_msgs::Landmark_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa44c2e55d48971b0ULL;
  static const uint64_t static_value2 = 0x74b6d4f211c997d7ULL;
};

template<class ContainerAllocator>
struct DataType< ::team_nust_msgs::Landmark_<ContainerAllocator> >
{
  static const char* value()
  {
    return "team_nust_msgs/Landmark";
  }

  static const char* value(const ::team_nust_msgs::Landmark_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::team_nust_msgs::Landmark_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 type\n\
geometry_msgs/Point pos\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::team_nust_msgs::Landmark_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::team_nust_msgs::Landmark_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
      stream.next(m.pos);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Landmark_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::team_nust_msgs::Landmark_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::team_nust_msgs::Landmark_<ContainerAllocator>& v)
  {
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
    s << indent << "pos: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.pos);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TEAM_NUST_MSGS_MESSAGE_LANDMARK_H
