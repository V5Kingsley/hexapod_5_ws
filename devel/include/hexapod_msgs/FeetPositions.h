// Generated by gencpp from file hexapod_msgs/FeetPositions.msg
// DO NOT EDIT!


#ifndef HEXAPOD_MSGS_MESSAGE_FEETPOSITIONS_H
#define HEXAPOD_MSGS_MESSAGE_FEETPOSITIONS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <hexapod_msgs/Pose.h>

namespace hexapod_msgs
{
template <class ContainerAllocator>
struct FeetPositions_
{
  typedef FeetPositions_<ContainerAllocator> Type;

  FeetPositions_()
    : foot()  {
    }
  FeetPositions_(const ContainerAllocator& _alloc)
    : foot()  {
  (void)_alloc;
      foot.assign( ::hexapod_msgs::Pose_<ContainerAllocator> (_alloc));
  }



   typedef boost::array< ::hexapod_msgs::Pose_<ContainerAllocator> , 6>  _foot_type;
  _foot_type foot;




  typedef boost::shared_ptr< ::hexapod_msgs::FeetPositions_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hexapod_msgs::FeetPositions_<ContainerAllocator> const> ConstPtr;

}; // struct FeetPositions_

typedef ::hexapod_msgs::FeetPositions_<std::allocator<void> > FeetPositions;

typedef boost::shared_ptr< ::hexapod_msgs::FeetPositions > FeetPositionsPtr;
typedef boost::shared_ptr< ::hexapod_msgs::FeetPositions const> FeetPositionsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hexapod_msgs::FeetPositions_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hexapod_msgs::FeetPositions_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace hexapod_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'hexapod_msgs': ['/home/sun/hexapod_5_ws/src/hexapod_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::hexapod_msgs::FeetPositions_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hexapod_msgs::FeetPositions_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hexapod_msgs::FeetPositions_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hexapod_msgs::FeetPositions_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hexapod_msgs::FeetPositions_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hexapod_msgs::FeetPositions_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hexapod_msgs::FeetPositions_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d12724c3b1519cfb275eb5b1d0e25de2";
  }

  static const char* value(const ::hexapod_msgs::FeetPositions_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd12724c3b1519cfbULL;
  static const uint64_t static_value2 = 0x275eb5b1d0e25de2ULL;
};

template<class ContainerAllocator>
struct DataType< ::hexapod_msgs::FeetPositions_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hexapod_msgs/FeetPositions";
  }

  static const char* value(const ::hexapod_msgs::FeetPositions_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hexapod_msgs::FeetPositions_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hexapod_msgs/Pose[6] foot\n\
\n\
================================================================================\n\
MSG: hexapod_msgs/Pose\n\
geometry_msgs/Point position\n\
hexapod_msgs/RPY orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: hexapod_msgs/RPY\n\
float64 roll\n\
float64 pitch\n\
float64 yaw\n\
";
  }

  static const char* value(const ::hexapod_msgs::FeetPositions_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hexapod_msgs::FeetPositions_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.foot);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FeetPositions_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hexapod_msgs::FeetPositions_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hexapod_msgs::FeetPositions_<ContainerAllocator>& v)
  {
    s << indent << "foot[]" << std::endl;
    for (size_t i = 0; i < v.foot.size(); ++i)
    {
      s << indent << "  foot[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::hexapod_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "    ", v.foot[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // HEXAPOD_MSGS_MESSAGE_FEETPOSITIONS_H
