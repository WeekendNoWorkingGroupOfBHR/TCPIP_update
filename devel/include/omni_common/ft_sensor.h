// Generated by gencpp from file omni_common/ft_sensor.msg
// DO NOT EDIT!


#ifndef OMNI_COMMON_MESSAGE_FT_SENSOR_H
#define OMNI_COMMON_MESSAGE_FT_SENSOR_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace omni_common
{
template <class ContainerAllocator>
struct ft_sensor_
{
  typedef ft_sensor_<ContainerAllocator> Type;

  ft_sensor_()
    : Fx(0.0)
    , Fy(0.0)
    , Fz(0.0)
    , Mx(0.0)
    , My(0.0)
    , Mz(0.0)  {
    }
  ft_sensor_(const ContainerAllocator& _alloc)
    : Fx(0.0)
    , Fy(0.0)
    , Fz(0.0)
    , Mx(0.0)
    , My(0.0)
    , Mz(0.0)  {
  (void)_alloc;
    }



   typedef float _Fx_type;
  _Fx_type Fx;

   typedef float _Fy_type;
  _Fy_type Fy;

   typedef float _Fz_type;
  _Fz_type Fz;

   typedef float _Mx_type;
  _Mx_type Mx;

   typedef float _My_type;
  _My_type My;

   typedef float _Mz_type;
  _Mz_type Mz;





  typedef boost::shared_ptr< ::omni_common::ft_sensor_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::omni_common::ft_sensor_<ContainerAllocator> const> ConstPtr;

}; // struct ft_sensor_

typedef ::omni_common::ft_sensor_<std::allocator<void> > ft_sensor;

typedef boost::shared_ptr< ::omni_common::ft_sensor > ft_sensorPtr;
typedef boost::shared_ptr< ::omni_common::ft_sensor const> ft_sensorConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::omni_common::ft_sensor_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::omni_common::ft_sensor_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::omni_common::ft_sensor_<ContainerAllocator1> & lhs, const ::omni_common::ft_sensor_<ContainerAllocator2> & rhs)
{
  return lhs.Fx == rhs.Fx &&
    lhs.Fy == rhs.Fy &&
    lhs.Fz == rhs.Fz &&
    lhs.Mx == rhs.Mx &&
    lhs.My == rhs.My &&
    lhs.Mz == rhs.Mz;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::omni_common::ft_sensor_<ContainerAllocator1> & lhs, const ::omni_common::ft_sensor_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace omni_common

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::omni_common::ft_sensor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::omni_common::ft_sensor_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::omni_common::ft_sensor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::omni_common::ft_sensor_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::omni_common::ft_sensor_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::omni_common::ft_sensor_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::omni_common::ft_sensor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "060fd5fcbaefcff9daec9d5bb2d41714";
  }

  static const char* value(const ::omni_common::ft_sensor_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x060fd5fcbaefcff9ULL;
  static const uint64_t static_value2 = 0xdaec9d5bb2d41714ULL;
};

template<class ContainerAllocator>
struct DataType< ::omni_common::ft_sensor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "omni_common/ft_sensor";
  }

  static const char* value(const ::omni_common::ft_sensor_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::omni_common::ft_sensor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 Fx\n"
"float32 Fy\n"
"float32 Fz\n"
"float32 Mx\n"
"float32 My\n"
"float32 Mz\n"
;
  }

  static const char* value(const ::omni_common::ft_sensor_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::omni_common::ft_sensor_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.Fx);
      stream.next(m.Fy);
      stream.next(m.Fz);
      stream.next(m.Mx);
      stream.next(m.My);
      stream.next(m.Mz);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ft_sensor_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::omni_common::ft_sensor_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::omni_common::ft_sensor_<ContainerAllocator>& v)
  {
    s << indent << "Fx: ";
    Printer<float>::stream(s, indent + "  ", v.Fx);
    s << indent << "Fy: ";
    Printer<float>::stream(s, indent + "  ", v.Fy);
    s << indent << "Fz: ";
    Printer<float>::stream(s, indent + "  ", v.Fz);
    s << indent << "Mx: ";
    Printer<float>::stream(s, indent + "  ", v.Mx);
    s << indent << "My: ";
    Printer<float>::stream(s, indent + "  ", v.My);
    s << indent << "Mz: ";
    Printer<float>::stream(s, indent + "  ", v.Mz);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OMNI_COMMON_MESSAGE_FT_SENSOR_H