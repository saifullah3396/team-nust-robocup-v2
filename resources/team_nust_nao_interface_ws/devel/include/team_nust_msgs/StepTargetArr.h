// Generated by gencpp from file team_nust_msgs/StepTargetArr.msg
// DO NOT EDIT!


#ifndef TEAM_NUST_MSGS_MESSAGE_STEPTARGETARR_H
#define TEAM_NUST_MSGS_MESSAGE_STEPTARGETARR_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <humanoid_nav_msgs/StepTarget.h>

namespace team_nust_msgs
{
template <class ContainerAllocator>
struct StepTargetArr_
{
  typedef StepTargetArr_<ContainerAllocator> Type;

  StepTargetArr_()
    : header()
    , steps()  {
    }
  StepTargetArr_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , steps(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> >::other >  _steps_type;
  _steps_type steps;




  typedef boost::shared_ptr< ::team_nust_msgs::StepTargetArr_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::team_nust_msgs::StepTargetArr_<ContainerAllocator> const> ConstPtr;

}; // struct StepTargetArr_

typedef ::team_nust_msgs::StepTargetArr_<std::allocator<void> > StepTargetArr;

typedef boost::shared_ptr< ::team_nust_msgs::StepTargetArr > StepTargetArrPtr;
typedef boost::shared_ptr< ::team_nust_msgs::StepTargetArr const> StepTargetArrConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::team_nust_msgs::StepTargetArr_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::team_nust_msgs::StepTargetArr_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace team_nust_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'team_nust_msgs': ['/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'humanoid_nav_msgs': ['/opt/ros/indigo/share/humanoid_nav_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::team_nust_msgs::StepTargetArr_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::team_nust_msgs::StepTargetArr_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::team_nust_msgs::StepTargetArr_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::team_nust_msgs::StepTargetArr_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::team_nust_msgs::StepTargetArr_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::team_nust_msgs::StepTargetArr_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::team_nust_msgs::StepTargetArr_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5d94e0eebae888055949c996dbe5e324";
  }

  static const char* value(const ::team_nust_msgs::StepTargetArr_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5d94e0eebae88805ULL;
  static const uint64_t static_value2 = 0x5949c996dbe5e324ULL;
};

template<class ContainerAllocator>
struct DataType< ::team_nust_msgs::StepTargetArr_<ContainerAllocator> >
{
  static const char* value()
  {
    return "team_nust_msgs/StepTargetArr";
  }

  static const char* value(const ::team_nust_msgs::StepTargetArr_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::team_nust_msgs::StepTargetArr_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
humanoid_nav_msgs/StepTarget[] steps\n\
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
\n\
================================================================================\n\
MSG: humanoid_nav_msgs/StepTarget\n\
# Target for a single stepping motion of a humanoid's leg\n\
\n\
geometry_msgs/Pose2D pose   # step pose as relative offset to last leg\n\
uint8 leg                   # which leg to use (left/right, see below)\n\
\n\
uint8 right=0               # right leg constant\n\
uint8 left=1                # left leg constant\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose2D\n\
# This expresses a position and orientation on a 2D manifold.\n\
\n\
float64 x\n\
float64 y\n\
float64 theta\n\
";
  }

  static const char* value(const ::team_nust_msgs::StepTargetArr_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::team_nust_msgs::StepTargetArr_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.steps);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StepTargetArr_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::team_nust_msgs::StepTargetArr_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::team_nust_msgs::StepTargetArr_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "steps[]" << std::endl;
    for (size_t i = 0; i < v.steps.size(); ++i)
    {
      s << indent << "  steps[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> >::stream(s, indent + "    ", v.steps[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // TEAM_NUST_MSGS_MESSAGE_STEPTARGETARR_H
