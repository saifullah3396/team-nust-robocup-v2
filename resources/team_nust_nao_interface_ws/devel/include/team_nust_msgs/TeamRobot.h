// Generated by gencpp from file team_nust_msgs/TeamRobot.msg
// DO NOT EDIT!


#ifndef TEAM_NUST_MSGS_MESSAGE_TEAMROBOT_H
#define TEAM_NUST_MSGS_MESSAGE_TEAMROBOT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>

namespace team_nust_msgs
{
template <class ContainerAllocator>
struct TeamRobot_
{
  typedef TeamRobot_<ContainerAllocator> Type;

  TeamRobot_()
    : data_received(false)
    , fallen(false)
    , intention(0)
    , suggestion_to_me(0)
    , pose_2d()
    , walking_to()
    , shooting_to()  {
    }
  TeamRobot_(const ContainerAllocator& _alloc)
    : data_received(false)
    , fallen(false)
    , intention(0)
    , suggestion_to_me(0)
    , pose_2d(_alloc)
    , walking_to(_alloc)
    , shooting_to(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _data_received_type;
  _data_received_type data_received;

   typedef uint8_t _fallen_type;
  _fallen_type fallen;

   typedef int32_t _intention_type;
  _intention_type intention;

   typedef int32_t _suggestion_to_me_type;
  _suggestion_to_me_type suggestion_to_me;

   typedef  ::geometry_msgs::Pose2D_<ContainerAllocator>  _pose_2d_type;
  _pose_2d_type pose_2d;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _walking_to_type;
  _walking_to_type walking_to;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _shooting_to_type;
  _shooting_to_type shooting_to;




  typedef boost::shared_ptr< ::team_nust_msgs::TeamRobot_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::team_nust_msgs::TeamRobot_<ContainerAllocator> const> ConstPtr;

}; // struct TeamRobot_

typedef ::team_nust_msgs::TeamRobot_<std::allocator<void> > TeamRobot;

typedef boost::shared_ptr< ::team_nust_msgs::TeamRobot > TeamRobotPtr;
typedef boost::shared_ptr< ::team_nust_msgs::TeamRobot const> TeamRobotConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::team_nust_msgs::TeamRobot_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::team_nust_msgs::TeamRobot_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace team_nust_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'team_nust_msgs': ['/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'humanoid_nav_msgs': ['/opt/ros/indigo/share/humanoid_nav_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::team_nust_msgs::TeamRobot_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::team_nust_msgs::TeamRobot_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::team_nust_msgs::TeamRobot_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::team_nust_msgs::TeamRobot_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::team_nust_msgs::TeamRobot_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::team_nust_msgs::TeamRobot_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::team_nust_msgs::TeamRobot_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8b91570e4f56d05a3dd7171dea9d4212";
  }

  static const char* value(const ::team_nust_msgs::TeamRobot_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8b91570e4f56d05aULL;
  static const uint64_t static_value2 = 0x3dd7171dea9d4212ULL;
};

template<class ContainerAllocator>
struct DataType< ::team_nust_msgs::TeamRobot_<ContainerAllocator> >
{
  static const char* value()
  {
    return "team_nust_msgs/TeamRobot";
  }

  static const char* value(const ::team_nust_msgs::TeamRobot_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::team_nust_msgs::TeamRobot_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool data_received\n\
bool fallen\n\
int32 intention\n\
int32 suggestion_to_me\n\
geometry_msgs/Pose2D pose_2d\n\
geometry_msgs/Point walking_to\n\
geometry_msgs/Point shooting_to\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose2D\n\
# This expresses a position and orientation on a 2D manifold.\n\
\n\
float64 x\n\
float64 y\n\
float64 theta\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::team_nust_msgs::TeamRobot_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::team_nust_msgs::TeamRobot_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data_received);
      stream.next(m.fallen);
      stream.next(m.intention);
      stream.next(m.suggestion_to_me);
      stream.next(m.pose_2d);
      stream.next(m.walking_to);
      stream.next(m.shooting_to);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TeamRobot_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::team_nust_msgs::TeamRobot_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::team_nust_msgs::TeamRobot_<ContainerAllocator>& v)
  {
    s << indent << "data_received: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.data_received);
    s << indent << "fallen: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.fallen);
    s << indent << "intention: ";
    Printer<int32_t>::stream(s, indent + "  ", v.intention);
    s << indent << "suggestion_to_me: ";
    Printer<int32_t>::stream(s, indent + "  ", v.suggestion_to_me);
    s << indent << "pose_2d: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose2D_<ContainerAllocator> >::stream(s, indent + "  ", v.pose_2d);
    s << indent << "walking_to: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.walking_to);
    s << indent << "shooting_to: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.shooting_to);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TEAM_NUST_MSGS_MESSAGE_TEAMROBOT_H
