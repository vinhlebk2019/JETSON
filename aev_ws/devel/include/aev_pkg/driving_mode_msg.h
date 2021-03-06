// Generated by gencpp from file aev_pkg/driving_mode_msg.msg
// DO NOT EDIT!


#ifndef AEV_PKG_MESSAGE_DRIVING_MODE_MSG_H
#define AEV_PKG_MESSAGE_DRIVING_MODE_MSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace aev_pkg
{
template <class ContainerAllocator>
struct driving_mode_msg_
{
  typedef driving_mode_msg_<ContainerAllocator> Type;

  driving_mode_msg_()
    : msg_counter(0)
    , drivingMode(0)  {
    }
  driving_mode_msg_(const ContainerAllocator& _alloc)
    : msg_counter(0)
    , drivingMode(0)  {
  (void)_alloc;
    }



   typedef uint32_t _msg_counter_type;
  _msg_counter_type msg_counter;

   typedef uint8_t _drivingMode_type;
  _drivingMode_type drivingMode;





  typedef boost::shared_ptr< ::aev_pkg::driving_mode_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::aev_pkg::driving_mode_msg_<ContainerAllocator> const> ConstPtr;

}; // struct driving_mode_msg_

typedef ::aev_pkg::driving_mode_msg_<std::allocator<void> > driving_mode_msg;

typedef boost::shared_ptr< ::aev_pkg::driving_mode_msg > driving_mode_msgPtr;
typedef boost::shared_ptr< ::aev_pkg::driving_mode_msg const> driving_mode_msgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::aev_pkg::driving_mode_msg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::aev_pkg::driving_mode_msg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::aev_pkg::driving_mode_msg_<ContainerAllocator1> & lhs, const ::aev_pkg::driving_mode_msg_<ContainerAllocator2> & rhs)
{
  return lhs.msg_counter == rhs.msg_counter &&
    lhs.drivingMode == rhs.drivingMode;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::aev_pkg::driving_mode_msg_<ContainerAllocator1> & lhs, const ::aev_pkg::driving_mode_msg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace aev_pkg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::aev_pkg::driving_mode_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::aev_pkg::driving_mode_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aev_pkg::driving_mode_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aev_pkg::driving_mode_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aev_pkg::driving_mode_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aev_pkg::driving_mode_msg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::aev_pkg::driving_mode_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a48a69dcaa1f71cf7c0fafc132da8148";
  }

  static const char* value(const ::aev_pkg::driving_mode_msg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa48a69dcaa1f71cfULL;
  static const uint64_t static_value2 = 0x7c0fafc132da8148ULL;
};

template<class ContainerAllocator>
struct DataType< ::aev_pkg::driving_mode_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "aev_pkg/driving_mode_msg";
  }

  static const char* value(const ::aev_pkg::driving_mode_msg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::aev_pkg::driving_mode_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 	msg_counter\n"
"uint8 	drivingMode\n"
;
  }

  static const char* value(const ::aev_pkg::driving_mode_msg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::aev_pkg::driving_mode_msg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.msg_counter);
      stream.next(m.drivingMode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct driving_mode_msg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::aev_pkg::driving_mode_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::aev_pkg::driving_mode_msg_<ContainerAllocator>& v)
  {
    s << indent << "msg_counter: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.msg_counter);
    s << indent << "drivingMode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.drivingMode);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AEV_PKG_MESSAGE_DRIVING_MODE_MSG_H
