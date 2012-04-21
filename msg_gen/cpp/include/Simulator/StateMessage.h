/* Auto-generated by genmsg_cpp for file /home/student/workspace/Simulator/msg/StateMessage.msg */
#ifndef SIMULATOR_MESSAGE_STATEMESSAGE_H
#define SIMULATOR_MESSAGE_STATEMESSAGE_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace Simulator
{
template <class ContainerAllocator>
struct StateMessage_ {
  typedef StateMessage_<ContainerAllocator> Type;

  StateMessage_()
  : angular_error(0.0)
  , timestep(0)
  , reference_id(0)
  , temperature(0.0)
  , heat(0.0)
  {
  }

  StateMessage_(const ContainerAllocator& _alloc)
  : angular_error(0.0)
  , timestep(0)
  , reference_id(0)
  , temperature(0.0)
  , heat(0.0)
  {
  }

  typedef double _angular_error_type;
  double angular_error;

  typedef int32_t _timestep_type;
  int32_t timestep;

  typedef int32_t _reference_id_type;
  int32_t reference_id;

  typedef double _temperature_type;
  double temperature;

  typedef double _heat_type;
  double heat;


private:
  static const char* __s_getDataType_() { return "Simulator/StateMessage"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "199c3c439236ac8d1a99f8080aafbf52"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "#Formation formation\n\
#Vector frp\n\
#Relationship[] relationships\n\
#Vector linear_error\n\
float64 angular_error\n\
int32 timestep\n\
int32 reference_id\n\
float64 temperature\n\
float64 heat\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, angular_error);
    ros::serialization::serialize(stream, timestep);
    ros::serialization::serialize(stream, reference_id);
    ros::serialization::serialize(stream, temperature);
    ros::serialization::serialize(stream, heat);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, angular_error);
    ros::serialization::deserialize(stream, timestep);
    ros::serialization::deserialize(stream, reference_id);
    ros::serialization::deserialize(stream, temperature);
    ros::serialization::deserialize(stream, heat);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(angular_error);
    size += ros::serialization::serializationLength(timestep);
    size += ros::serialization::serializationLength(reference_id);
    size += ros::serialization::serializationLength(temperature);
    size += ros::serialization::serializationLength(heat);
    return size;
  }

  typedef boost::shared_ptr< ::Simulator::StateMessage_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::Simulator::StateMessage_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct StateMessage
typedef  ::Simulator::StateMessage_<std::allocator<void> > StateMessage;

typedef boost::shared_ptr< ::Simulator::StateMessage> StateMessagePtr;
typedef boost::shared_ptr< ::Simulator::StateMessage const> StateMessageConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::Simulator::StateMessage_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::Simulator::StateMessage_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace Simulator

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::Simulator::StateMessage_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::Simulator::StateMessage_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::Simulator::StateMessage_<ContainerAllocator> > {
  static const char* value() 
  {
    return "199c3c439236ac8d1a99f8080aafbf52";
  }

  static const char* value(const  ::Simulator::StateMessage_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x199c3c439236ac8dULL;
  static const uint64_t static_value2 = 0x1a99f8080aafbf52ULL;
};

template<class ContainerAllocator>
struct DataType< ::Simulator::StateMessage_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Simulator/StateMessage";
  }

  static const char* value(const  ::Simulator::StateMessage_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::Simulator::StateMessage_<ContainerAllocator> > {
  static const char* value() 
  {
    return "#Formation formation\n\
#Vector frp\n\
#Relationship[] relationships\n\
#Vector linear_error\n\
float64 angular_error\n\
int32 timestep\n\
int32 reference_id\n\
float64 temperature\n\
float64 heat\n\
";
  }

  static const char* value(const  ::Simulator::StateMessage_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::Simulator::StateMessage_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::Simulator::StateMessage_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.angular_error);
    stream.next(m.timestep);
    stream.next(m.reference_id);
    stream.next(m.temperature);
    stream.next(m.heat);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct StateMessage_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::Simulator::StateMessage_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::Simulator::StateMessage_<ContainerAllocator> & v) 
  {
    s << indent << "angular_error: ";
    Printer<double>::stream(s, indent + "  ", v.angular_error);
    s << indent << "timestep: ";
    Printer<int32_t>::stream(s, indent + "  ", v.timestep);
    s << indent << "reference_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.reference_id);
    s << indent << "temperature: ";
    Printer<double>::stream(s, indent + "  ", v.temperature);
    s << indent << "heat: ";
    Printer<double>::stream(s, indent + "  ", v.heat);
  }
};


} // namespace message_operations
} // namespace ros

#endif // SIMULATOR_MESSAGE_STATEMESSAGE_H

