// Generated by gencpp from file pkg_ros_basics/AddTwoIntsRequest.msg
// DO NOT EDIT!


#ifndef PKG_ROS_BASICS_MESSAGE_ADDTWOINTSREQUEST_H
#define PKG_ROS_BASICS_MESSAGE_ADDTWOINTSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pkg_ros_basics
{
template <class ContainerAllocator>
struct AddTwoIntsRequest_
{
  typedef AddTwoIntsRequest_<ContainerAllocator> Type;

  AddTwoIntsRequest_()
    : A(0)
    , B(0)  {
    }
  AddTwoIntsRequest_(const ContainerAllocator& _alloc)
    : A(0)
    , B(0)  {
  (void)_alloc;
    }



   typedef int64_t _A_type;
  _A_type A;

   typedef int64_t _B_type;
  _B_type B;





  typedef boost::shared_ptr< ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct AddTwoIntsRequest_

typedef ::pkg_ros_basics::AddTwoIntsRequest_<std::allocator<void> > AddTwoIntsRequest;

typedef boost::shared_ptr< ::pkg_ros_basics::AddTwoIntsRequest > AddTwoIntsRequestPtr;
typedef boost::shared_ptr< ::pkg_ros_basics::AddTwoIntsRequest const> AddTwoIntsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator1> & lhs, const ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator2> & rhs)
{
  return lhs.A == rhs.A &&
    lhs.B == rhs.B;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator1> & lhs, const ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pkg_ros_basics

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "02f05bb5bde9ff0aeeed7cca0d2e13fc";
  }

  static const char* value(const ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x02f05bb5bde9ff0aULL;
  static const uint64_t static_value2 = 0xeeed7cca0d2e13fcULL;
};

template<class ContainerAllocator>
struct DataType< ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pkg_ros_basics/AddTwoIntsRequest";
  }

  static const char* value(const ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64 A\n"
"int64 B\n"
;
  }

  static const char* value(const ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.A);
      stream.next(m.B);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AddTwoIntsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pkg_ros_basics::AddTwoIntsRequest_<ContainerAllocator>& v)
  {
    s << indent << "A: ";
    Printer<int64_t>::stream(s, indent + "  ", v.A);
    s << indent << "B: ";
    Printer<int64_t>::stream(s, indent + "  ", v.B);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PKG_ROS_BASICS_MESSAGE_ADDTWOINTSREQUEST_H