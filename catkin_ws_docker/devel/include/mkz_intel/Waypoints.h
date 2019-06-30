// Generated by gencpp from file mkz_intel/Waypoints.msg
// DO NOT EDIT!


#ifndef MKZ_INTEL_MESSAGE_WAYPOINTS_H
#define MKZ_INTEL_MESSAGE_WAYPOINTS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <mkz_intel/Point2D.h>

namespace mkz_intel
{
template <class ContainerAllocator>
struct Waypoints_
{
  typedef Waypoints_<ContainerAllocator> Type;

  Waypoints_()
    : points()
    , dt(0.0)  {
    }
  Waypoints_(const ContainerAllocator& _alloc)
    : points(_alloc)
    , dt(0.0)  {
  (void)_alloc;
    }



   typedef std::vector< ::mkz_intel::Point2D_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::mkz_intel::Point2D_<ContainerAllocator> >::other >  _points_type;
  _points_type points;

   typedef float _dt_type;
  _dt_type dt;





  typedef boost::shared_ptr< ::mkz_intel::Waypoints_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mkz_intel::Waypoints_<ContainerAllocator> const> ConstPtr;

}; // struct Waypoints_

typedef ::mkz_intel::Waypoints_<std::allocator<void> > Waypoints;

typedef boost::shared_ptr< ::mkz_intel::Waypoints > WaypointsPtr;
typedef boost::shared_ptr< ::mkz_intel::Waypoints const> WaypointsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mkz_intel::Waypoints_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mkz_intel::Waypoints_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace mkz_intel

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'mkz_intel': ['/data/yang/code/aws/catkin_ws_docker/src/mkz_intel/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'nmea_msgs': ['/opt/ros/kinetic/share/nmea_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::mkz_intel::Waypoints_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mkz_intel::Waypoints_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mkz_intel::Waypoints_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mkz_intel::Waypoints_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mkz_intel::Waypoints_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mkz_intel::Waypoints_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mkz_intel::Waypoints_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dfb75f45f26568fb46a75f56fa6b81e2";
  }

  static const char* value(const ::mkz_intel::Waypoints_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdfb75f45f26568fbULL;
  static const uint64_t static_value2 = 0x46a75f56fa6b81e2ULL;
};

template<class ContainerAllocator>
struct DataType< ::mkz_intel::Waypoints_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mkz_intel/Waypoints";
  }

  static const char* value(const ::mkz_intel::Waypoints_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mkz_intel::Waypoints_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Point2D[] points\n\
float32 dt\n\
\n\
================================================================================\n\
MSG: mkz_intel/Point2D\n\
float64 x\n\
float64 y\n\
";
  }

  static const char* value(const ::mkz_intel::Waypoints_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mkz_intel::Waypoints_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.points);
      stream.next(m.dt);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Waypoints_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mkz_intel::Waypoints_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mkz_intel::Waypoints_<ContainerAllocator>& v)
  {
    s << indent << "points[]" << std::endl;
    for (size_t i = 0; i < v.points.size(); ++i)
    {
      s << indent << "  points[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::mkz_intel::Point2D_<ContainerAllocator> >::stream(s, indent + "    ", v.points[i]);
    }
    s << indent << "dt: ";
    Printer<float>::stream(s, indent + "  ", v.dt);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MKZ_INTEL_MESSAGE_WAYPOINTS_H
