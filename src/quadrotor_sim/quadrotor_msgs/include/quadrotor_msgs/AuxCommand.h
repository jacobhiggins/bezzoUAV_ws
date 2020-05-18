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
 * Auto-generated by genmsg_cpp from file /home/precise/duyanwei/quadrotor_ws/src/quadrotor_msgs/msg/AuxCommand.msg
 *
 */
// ----------------------------------------------------------
// Code by: Yanwei Du, Nicola Bezzo (nbezzo@gmail.com)
//          PRECISE Center
// Date: 09/07/2015
// DO NOT PUBLISH OR SHARE unless authorized by the Authors 
//                         of the code
// ----------------------------------------------------------

#ifndef QUADROTOR_MSGS_MESSAGE_AUXCOMMAND_H
#define QUADROTOR_MSGS_MESSAGE_AUXCOMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace quadrotor_msgs
{
template <class ContainerAllocator>
struct AuxCommand_
{
  typedef AuxCommand_<ContainerAllocator> Type;

  AuxCommand_()
    : current_yaw(0.0)
    , kf_correction(0.0)
    , angle_corrections()
    , enable_motors(false)
    , use_external_yaw(false)  {
      angle_corrections.assign(0.0);
  }
  AuxCommand_(const ContainerAllocator& _alloc)
    : current_yaw(0.0)
    , kf_correction(0.0)
    , angle_corrections()
    , enable_motors(false)
    , use_external_yaw(false)  {
      angle_corrections.assign(0.0);
  }



   typedef double _current_yaw_type;
  _current_yaw_type current_yaw;

   typedef double _kf_correction_type;
  _kf_correction_type kf_correction;

   typedef boost::array<double, 2>  _angle_corrections_type;
  _angle_corrections_type angle_corrections;

   typedef uint8_t _enable_motors_type;
  _enable_motors_type enable_motors;

   typedef uint8_t _use_external_yaw_type;
  _use_external_yaw_type use_external_yaw;




  typedef boost::shared_ptr< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct AuxCommand_

typedef ::quadrotor_msgs::AuxCommand_<std::allocator<void> > AuxCommand;

typedef boost::shared_ptr< ::quadrotor_msgs::AuxCommand > AuxCommandPtr;
typedef boost::shared_ptr< ::quadrotor_msgs::AuxCommand const> AuxCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::quadrotor_msgs::AuxCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace quadrotor_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/groovy/share/geometry_msgs/cmake/../msg'], 'quadrotor_msgs': ['/home/precise/duyanwei/quadrotor_ws/src/quadrotor_msgs/msg', '/home/precise/duyanwei/quadrotor_ws/src/quadrotor_msgs/msg'], 'std_msgs': ['/opt/ros/groovy/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "94f75840e4b1e03675da764692f2c839";
  }

  static const char* value(const ::quadrotor_msgs::AuxCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x94f75840e4b1e036ULL;
  static const uint64_t static_value2 = 0x75da764692f2c839ULL;
};

template<class ContainerAllocator>
struct DataType< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "quadrotor_msgs/AuxCommand";
  }

  static const char* value(const ::quadrotor_msgs::AuxCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 current_yaw\n\
float64 kf_correction\n\
float64[2] angle_corrections\n\
bool enable_motors\n\
bool use_external_yaw\n\
";
  }

  static const char* value(const ::quadrotor_msgs::AuxCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.current_yaw);
      stream.next(m.kf_correction);
      stream.next(m.angle_corrections);
      stream.next(m.enable_motors);
      stream.next(m.use_external_yaw);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct AuxCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::quadrotor_msgs::AuxCommand_<ContainerAllocator>& v)
  {
    s << indent << "current_yaw: ";
    Printer<double>::stream(s, indent + "  ", v.current_yaw);
    s << indent << "kf_correction: ";
    Printer<double>::stream(s, indent + "  ", v.kf_correction);
    s << indent << "angle_corrections[]" << std::endl;
    for (size_t i = 0; i < v.angle_corrections.size(); ++i)
    {
      s << indent << "  angle_corrections[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.angle_corrections[i]);
    }
    s << indent << "enable_motors: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.enable_motors);
    s << indent << "use_external_yaw: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_external_yaw);
  }
};

} // namespace message_operations
} // namespace ros

#endif // QUADROTOR_MSGS_MESSAGE_AUXCOMMAND_H