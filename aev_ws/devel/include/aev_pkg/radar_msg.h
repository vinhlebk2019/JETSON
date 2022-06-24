// Generated by gencpp from file aev_pkg/radar_msg.msg
// DO NOT EDIT!


#ifndef AEV_PKG_MESSAGE_RADAR_MSG_H
#define AEV_PKG_MESSAGE_RADAR_MSG_H


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
struct radar_msg_
{
  typedef radar_msg_<ContainerAllocator> Type;

  radar_msg_()
    : numObj(0)
    , IdObj()
    , isApproach()
    , alpha()
    , posX()
    , posY()
    , dis()
    , vel()
    , ttc()
    , safetyZone()
    , msg_counter(0)
    , isObject(false)
    , distance(0.0)  {
    }
  radar_msg_(const ContainerAllocator& _alloc)
    : numObj(0)
    , IdObj(_alloc)
    , isApproach(_alloc)
    , alpha(_alloc)
    , posX(_alloc)
    , posY(_alloc)
    , dis(_alloc)
    , vel(_alloc)
    , ttc(_alloc)
    , safetyZone(_alloc)
    , msg_counter(0)
    , isObject(false)
    , distance(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _numObj_type;
  _numObj_type numObj;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _IdObj_type;
  _IdObj_type IdObj;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _isApproach_type;
  _isApproach_type isApproach;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _alpha_type;
  _alpha_type alpha;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _posX_type;
  _posX_type posX;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _posY_type;
  _posY_type posY;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _dis_type;
  _dis_type dis;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _vel_type;
  _vel_type vel;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _ttc_type;
  _ttc_type ttc;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _safetyZone_type;
  _safetyZone_type safetyZone;

   typedef uint32_t _msg_counter_type;
  _msg_counter_type msg_counter;

   typedef uint8_t _isObject_type;
  _isObject_type isObject;

   typedef float _distance_type;
  _distance_type distance;





  typedef boost::shared_ptr< ::aev_pkg::radar_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::aev_pkg::radar_msg_<ContainerAllocator> const> ConstPtr;

}; // struct radar_msg_

typedef ::aev_pkg::radar_msg_<std::allocator<void> > radar_msg;

typedef boost::shared_ptr< ::aev_pkg::radar_msg > radar_msgPtr;
typedef boost::shared_ptr< ::aev_pkg::radar_msg const> radar_msgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::aev_pkg::radar_msg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::aev_pkg::radar_msg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::aev_pkg::radar_msg_<ContainerAllocator1> & lhs, const ::aev_pkg::radar_msg_<ContainerAllocator2> & rhs)
{
  return lhs.numObj == rhs.numObj &&
    lhs.IdObj == rhs.IdObj &&
    lhs.isApproach == rhs.isApproach &&
    lhs.alpha == rhs.alpha &&
    lhs.posX == rhs.posX &&
    lhs.posY == rhs.posY &&
    lhs.dis == rhs.dis &&
    lhs.vel == rhs.vel &&
    lhs.ttc == rhs.ttc &&
    lhs.safetyZone == rhs.safetyZone &&
    lhs.msg_counter == rhs.msg_counter &&
    lhs.isObject == rhs.isObject &&
    lhs.distance == rhs.distance;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::aev_pkg::radar_msg_<ContainerAllocator1> & lhs, const ::aev_pkg::radar_msg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace aev_pkg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::aev_pkg::radar_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::aev_pkg::radar_msg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aev_pkg::radar_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aev_pkg::radar_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aev_pkg::radar_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aev_pkg::radar_msg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::aev_pkg::radar_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "68796b4398ded33c3293e6153473810f";
  }

  static const char* value(const ::aev_pkg::radar_msg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x68796b4398ded33cULL;
  static const uint64_t static_value2 = 0x3293e6153473810fULL;
};

template<class ContainerAllocator>
struct DataType< ::aev_pkg::radar_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "aev_pkg/radar_msg";
  }

  static const char* value(const ::aev_pkg::radar_msg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::aev_pkg::radar_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 numObj\n"
"uint8[] IdObj\n"
"bool[] isApproach\n"
"float32[] alpha\n"
"float32[] posX\n"
"float32[] posY\n"
"float32[] dis\n"
"float32[] vel\n"
"float32[] ttc\n"
"string[] safetyZone\n"
"\n"
"uint32 msg_counter\n"
"bool isObject\n"
"float32 distance\n"
;
  }

  static const char* value(const ::aev_pkg::radar_msg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::aev_pkg::radar_msg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.numObj);
      stream.next(m.IdObj);
      stream.next(m.isApproach);
      stream.next(m.alpha);
      stream.next(m.posX);
      stream.next(m.posY);
      stream.next(m.dis);
      stream.next(m.vel);
      stream.next(m.ttc);
      stream.next(m.safetyZone);
      stream.next(m.msg_counter);
      stream.next(m.isObject);
      stream.next(m.distance);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct radar_msg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::aev_pkg::radar_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::aev_pkg::radar_msg_<ContainerAllocator>& v)
  {
    s << indent << "numObj: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.numObj);
    s << indent << "IdObj[]" << std::endl;
    for (size_t i = 0; i < v.IdObj.size(); ++i)
    {
      s << indent << "  IdObj[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.IdObj[i]);
    }
    s << indent << "isApproach[]" << std::endl;
    for (size_t i = 0; i < v.isApproach.size(); ++i)
    {
      s << indent << "  isApproach[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.isApproach[i]);
    }
    s << indent << "alpha[]" << std::endl;
    for (size_t i = 0; i < v.alpha.size(); ++i)
    {
      s << indent << "  alpha[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.alpha[i]);
    }
    s << indent << "posX[]" << std::endl;
    for (size_t i = 0; i < v.posX.size(); ++i)
    {
      s << indent << "  posX[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.posX[i]);
    }
    s << indent << "posY[]" << std::endl;
    for (size_t i = 0; i < v.posY.size(); ++i)
    {
      s << indent << "  posY[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.posY[i]);
    }
    s << indent << "dis[]" << std::endl;
    for (size_t i = 0; i < v.dis.size(); ++i)
    {
      s << indent << "  dis[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.dis[i]);
    }
    s << indent << "vel[]" << std::endl;
    for (size_t i = 0; i < v.vel.size(); ++i)
    {
      s << indent << "  vel[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.vel[i]);
    }
    s << indent << "ttc[]" << std::endl;
    for (size_t i = 0; i < v.ttc.size(); ++i)
    {
      s << indent << "  ttc[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.ttc[i]);
    }
    s << indent << "safetyZone[]" << std::endl;
    for (size_t i = 0; i < v.safetyZone.size(); ++i)
    {
      s << indent << "  safetyZone[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.safetyZone[i]);
    }
    s << indent << "msg_counter: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.msg_counter);
    s << indent << "isObject: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.isObject);
    s << indent << "distance: ";
    Printer<float>::stream(s, indent + "  ", v.distance);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AEV_PKG_MESSAGE_RADAR_MSG_H