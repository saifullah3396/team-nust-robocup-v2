// Generated by gencpp from file team_nust_msgs/ObsLandmarks.msg
// DO NOT EDIT!


#ifndef TEAM_NUST_MSGS_MESSAGE_OBSLANDMARKS_H
#define TEAM_NUST_MSGS_MESSAGE_OBSLANDMARKS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <team_nust_msgs/Landmark.h>

namespace team_nust_msgs
{
template <class ContainerAllocator>
struct ObsLandmarks_
{
  typedef ObsLandmarks_<ContainerAllocator> Type;

  ObsLandmarks_()
    : header()
    , landmarks()  {
    }
  ObsLandmarks_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , landmarks(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::team_nust_msgs::Landmark_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::team_nust_msgs::Landmark_<ContainerAllocator> >::other >  _landmarks_type;
  _landmarks_type landmarks;




  typedef boost::shared_ptr< ::team_nust_msgs::ObsLandmarks_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::team_nust_msgs::ObsLandmarks_<ContainerAllocator> const> ConstPtr;

}; // struct ObsLandmarks_

typedef ::team_nust_msgs::ObsLandmarks_<std::allocator<void> > ObsLandmarks;

typedef boost::shared_ptr< ::team_nust_msgs::ObsLandmarks > ObsLandmarksPtr;
typedef boost::shared_ptr< ::team_nust_msgs::ObsLandmarks const> ObsLandmarksConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::team_nust_msgs::ObsLandmarks_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::team_nust_msgs::ObsLandmarks_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace team_nust_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'team_nust_msgs': ['/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'humanoid_nav_msgs': ['/opt/ros/indigo/share/humanoid_nav_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::team_nust_msgs::ObsLandmarks_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::team_nust_msgs::ObsLandmarks_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::team_nust_msgs::ObsLandmarks_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::team_nust_msgs::ObsLandmarks_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::team_nust_msgs::ObsLandmarks_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::team_nust_msgs::ObsLandmarks_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::team_nust_msgs::ObsLandmarks_<ContainerAllocator> >
{
  static const char* value()
  {
    return "71ddf4bd513db27a5697d59d383562d2";
  }

  static const char* value(const ::team_nust_msgs::ObsLandmarks_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x71ddf4bd513db27aULL;
  static const uint64_t static_value2 = 0x5697d59d383562d2ULL;
};

template<class ContainerAllocator>
struct DataType< ::team_nust_msgs::ObsLandmarks_<ContainerAllocator> >
{
  static const char* value()
  {
    return "team_nust_msgs/ObsLandmarks";
  }

  static const char* value(const ::team_nust_msgs::ObsLandmarks_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::team_nust_msgs::ObsLandmarks_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
Landmark[] landmarks\n\
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
MSG: team_nust_msgs/Landmark\n\
uint8 type\n\
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

  static const char* value(const ::team_nust_msgs::ObsLandmarks_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::team_nust_msgs::ObsLandmarks_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.landmarks);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ObsLandmarks_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::team_nust_msgs::ObsLandmarks_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::team_nust_msgs::ObsLandmarks_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "landmarks[]" << std::endl;
    for (size_t i = 0; i < v.landmarks.size(); ++i)
    {
      s << indent << "  landmarks[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::team_nust_msgs::Landmark_<ContainerAllocator> >::stream(s, indent + "    ", v.landmarks[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // TEAM_NUST_MSGS_MESSAGE_OBSLANDMARKS_H
