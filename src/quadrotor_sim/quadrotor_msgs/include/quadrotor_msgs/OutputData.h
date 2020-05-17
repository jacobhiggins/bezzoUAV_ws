/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/precise/duyanwei/quadrotor_ws/src/quadrotor_msgs/msg/OutputData.msg
 *
 */
// ----------------------------------------------------------
// Code by: Yanwei Du, Nicola Bezzo (nbezzo@gmail.com)
//          PRECISE Center
// Date: 09/07/2015
// DO NOT PUBLISH OR SHARE unless authorized by the Authors 
//                         of the code
// ----------------------------------------------------------

#ifndef QUADROTOR_MSGS_MESSAGE_OUTPUTDATA_H
#define QUADROTOR_MSGS_MESSAGE_OUTPUTDATA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>

namespace quadrotor_msgs
{
template <class ContainerAllocator>
struct OutputData_
{
  typedef OutputData_<ContainerAllocator> Type;

  OutputData_()
    : header()
    , loop_rate(0)
    , voltage(0.0)
    , orientation()
    , angular_velocity()
    , linear_acceleration()
    , pressure_dheight(0.0)
    , pressure_height(0.0)
    , magnetic_field()
    , radio_channel()
    , seq(0)  {
      radio_channel.assign(0);
  }
  OutputData_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , loop_rate(0)
    , voltage(0.0)
    , orientation(_alloc)
    , angular_velocity(_alloc)
    , linear_acceleration(_alloc)
    , pressure_dheight(0.0)
    , pressure_height(0.0)
    , magnetic_field(_alloc)
    , radio_channel()
    , seq(0)  {
      radio_channel.assign(0);
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint16_t _loop_rate_type;
  _loop_rate_type loop_rate;

   typedef double _voltage_type;
  _voltage_type voltage;

   typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _orientation_type;
  _orientation_type orientation;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _angular_velocity_type;
  _angular_velocity_type angular_velocity;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _linear_acceleration_type;
  _linear_acceleration_type linear_acceleration;

   typedef double _pressure_dheight_type;
  _pressure_dheight_type pressure_dheight;

   typedef double _pressure_height_type;
  _pressure_height_type pressure_height;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _magnetic_field_type;
  _magnetic_field_type magnetic_field;

   typedef boost::array<uint8_t, 8>  _radio_channel_type;
  _radio_channel_type radio_channel;

   typedef uint8_t _seq_type;
  _seq_type seq;




  typedef boost::shared_ptr< ::quadrotor_msgs::OutputData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quadrotor_msgs::OutputData_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct OutputData_

typedef ::quadrotor_msgs::OutputData_<std::allocator<void> > OutputData;

typedef boost::shared_ptr< ::quadrotor_msgs::OutputData > OutputDataPtr;
typedef boost::shared_ptr< ::quadrotor_msgs::OutputData const> OutputDataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::quadrotor_msgs::OutputData_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::quadrotor_msgs::OutputData_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace quadrotor_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/groovy/share/geometry_msgs/cmake/../msg'], 'quadrotor_msgs': ['/home/precise/duyanwei/quadrotor_ws/src/quadrotor_msgs/msg', '/home/precise/duyanwei/quadrotor_ws/src/quadrotor_msgs/msg'], 'std_msgs': ['/opt/ros/groovy/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::quadrotor_msgs::OutputData_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::quadrotor_msgs::OutputData_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::OutputData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::OutputData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::OutputData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::OutputData_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::quadrotor_msgs::OutputData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5759ee97fd5c090dcdccf7fc3e50d024";
  }

  static const char* value(const ::quadrotor_msgs::OutputData_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5759ee97fd5c090dULL;
  static const uint64_t static_value2 = 0xcdccf7fc3e50d024ULL;
};

template<class ContainerAllocator>
struct DataType< ::quadrotor_msgs::OutputData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "quadrotor_msgs/OutputData";
  }

  static const char* value(const ::quadrotor_msgs::OutputData_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::quadrotor_msgs::OutputData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
uint16 loop_rate\n\
float64 voltage\n\
geometry_msgs/Quaternion orientation\n\
geometry_msgs/Vector3 angular_velocity\n\
geometry_msgs/Vector3 linear_acceleration\n\
float64 pressure_dheight\n\
float64 pressure_height\n\
geometry_msgs/Vector3 magnetic_field\n\
uint8[8] radio_channel\n\
uint8 seq\n\
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
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::quadrotor_msgs::OutputData_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::quadrotor_msgs::OutputData_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.loop_rate);
      stream.next(m.voltage);
      stream.next(m.orientation);
      stream.next(m.angular_velocity);
      stream.next(m.linear_acceleration);
      stream.next(m.pressure_dheight);
      stream.next(m.pressure_height);
      stream.next(m.magnetic_field);
      stream.next(m.radio_channel);
      stream.next(m.seq);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct OutputData_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::quadrotor_msgs::OutputData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::quadrotor_msgs::OutputData_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "loop_rate: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.loop_rate);
    s << indent << "voltage: ";
    Printer<double>::stream(s, indent + "  ", v.voltage);
    s << indent << "orientation: ";
    s << std::endl;
    Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(s, indent + "  ", v.orientation);
    s << indent << "angular_velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.angular_velocity);
    s << indent << "linear_acceleration: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.linear_acceleration);
    s << indent << "pressure_dheight: ";
    Printer<double>::stream(s, indent + "  ", v.pressure_dheight);
    s << indent << "pressure_height: ";
    Printer<double>::stream(s, indent + "  ", v.pressure_height);
    s << indent << "magnetic_field: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.magnetic_field);
    s << indent << "radio_channel[]" << std::endl;
    for (size_t i = 0; i < v.radio_channel.size(); ++i)
    {
      s << indent << "  radio_channel[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.radio_channel[i]);
    }
    s << indent << "seq: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.seq);
  }
};

} // namespace message_operations
} // namespace ros

#endif // QUADROTOR_MSGS_MESSAGE_OUTPUTDATA_H
