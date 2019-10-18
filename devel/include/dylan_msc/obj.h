// Generated by gencpp from file dylan_msc/obj.msg
// DO NOT EDIT!


#ifndef DYLAN_MSC_MESSAGE_OBJ_H
#define DYLAN_MSC_MESSAGE_OBJ_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>

namespace dylan_msc
{
template <class ContainerAllocator>
struct obj_
{
  typedef obj_<ContainerAllocator> Type;

  obj_()
    : index(0)
    , centroid()
    , min()
    , max()  {
    }
  obj_(const ContainerAllocator& _alloc)
    : index(0)
    , centroid(_alloc)
    , min(_alloc)
    , max(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _index_type;
  _index_type index;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _centroid_type;
  _centroid_type centroid;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _min_type;
  _min_type min;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _max_type;
  _max_type max;





  typedef boost::shared_ptr< ::dylan_msc::obj_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dylan_msc::obj_<ContainerAllocator> const> ConstPtr;

}; // struct obj_

typedef ::dylan_msc::obj_<std::allocator<void> > obj;

typedef boost::shared_ptr< ::dylan_msc::obj > objPtr;
typedef boost::shared_ptr< ::dylan_msc::obj const> objConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dylan_msc::obj_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dylan_msc::obj_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dylan_msc

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'dylan_msc': ['/home/dylan/catkin_ws/src/dylan_msc/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dylan_msc::obj_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dylan_msc::obj_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dylan_msc::obj_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dylan_msc::obj_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dylan_msc::obj_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dylan_msc::obj_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dylan_msc::obj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "39f905263ec0534b03a70af1d99b67f0";
  }

  static const char* value(const ::dylan_msc::obj_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x39f905263ec0534bULL;
  static const uint64_t static_value2 = 0x03a70af1d99b67f0ULL;
};

template<class ContainerAllocator>
struct DataType< ::dylan_msc::obj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dylan_msc/obj";
  }

  static const char* value(const ::dylan_msc::obj_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dylan_msc::obj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 index\n\
geometry_msgs/Point centroid\n\
geometry_msgs/Point min\n\
geometry_msgs/Point max\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::dylan_msc::obj_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dylan_msc::obj_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.index);
      stream.next(m.centroid);
      stream.next(m.min);
      stream.next(m.max);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct obj_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dylan_msc::obj_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dylan_msc::obj_<ContainerAllocator>& v)
  {
    s << indent << "index: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.index);
    s << indent << "centroid: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.centroid);
    s << indent << "min: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.min);
    s << indent << "max: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.max);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DYLAN_MSC_MESSAGE_OBJ_H