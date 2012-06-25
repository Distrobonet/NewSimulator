///* Auto-generated by genmsg_cpp for file /home/student/workspace/Simulator/srv/FormationIndex.srv */
//#ifndef SIMULATOR_SERVICE_FORMATIONINDEX_H
//#define SIMULATOR_SERVICE_FORMATIONINDEX_H
//#include <string>
//#include <vector>
//#include <map>
//#include <ostream>
//#include "ros/serialization.h"
//#include "ros/builtin_message_traits.h"
//#include "ros/message_operations.h"
//#include "ros/time.h"
//
//#include "ros/macros.h"
//
//#include "ros/assert.h"
//
//#include "ros/service_traits.h"
//
//
//
//
//namespace Simulator
//{
//template <class ContainerAllocator>
//struct FormationIndexRequest_ {
//  typedef FormationIndexRequest_<ContainerAllocator> Type;
//
//  FormationIndexRequest_()
//  {
//  }
//
//  FormationIndexRequest_(const ContainerAllocator& _alloc)
//  {
//  }
//
//
//private:
//  static const char* __s_getDataType_() { return "Simulator/FormationIndexRequest"; }
//public:
//  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }
//
//  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }
//
//private:
//  static const char* __s_getMD5Sum_() { return "d41d8cd98f00b204e9800998ecf8427e"; }
//public:
//  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }
//
//  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }
//
//private:
//  static const char* __s_getServerMD5Sum_() { return "3e0302d241547a13e5cebd81ed40641e"; }
//public:
//  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }
//
//  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }
//
//private:
//  static const char* __s_getMessageDefinition_() { return "\n\
//\n\
//"; }
//public:
//  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }
//
//  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }
//
//  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
//  {
//    ros::serialization::OStream stream(write_ptr, 1000000000);
//    return stream.getData();
//  }
//
//  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
//  {
//    ros::serialization::IStream stream(read_ptr, 1000000000);
//    return stream.getData();
//  }
//
//  ROS_DEPRECATED virtual uint32_t serializationLength() const
//  {
//    uint32_t size = 0;
//    return size;
//  }
//
//  typedef boost::shared_ptr< ::Simulator::FormationIndexRequest_<ContainerAllocator> > Ptr;
//  typedef boost::shared_ptr< ::Simulator::FormationIndexRequest_<ContainerAllocator>  const> ConstPtr;
//  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
//}; // struct FormationIndexRequest
//typedef  ::Simulator::FormationIndexRequest_<std::allocator<void> > FormationIndexRequest;
//
//typedef boost::shared_ptr< ::Simulator::FormationIndexRequest> FormationIndexRequestPtr;
//typedef boost::shared_ptr< ::Simulator::FormationIndexRequest const> FormationIndexRequestConstPtr;
//
//
//template <class ContainerAllocator>
//struct FormationIndexResponse_ {
//  typedef FormationIndexResponse_<ContainerAllocator> Type;
//
//  FormationIndexResponse_()
//  : formationIndex(0)
//  {
//  }
//
//  FormationIndexResponse_(const ContainerAllocator& _alloc)
//  : formationIndex(0)
//  {
//  }
//
//  typedef int64_t _formationIndex_type;
//  int64_t formationIndex;
//
//
//private:
//  static const char* __s_getDataType_() { return "Simulator/FormationIndexResponse"; }
//public:
//  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }
//
//  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }
//
//private:
//  static const char* __s_getMD5Sum_() { return "3e0302d241547a13e5cebd81ed40641e"; }
//public:
//  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }
//
//  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }
//
//private:
//  static const char* __s_getServerMD5Sum_() { return "3e0302d241547a13e5cebd81ed40641e"; }
//public:
//  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }
//
//  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }
//
//private:
//  static const char* __s_getMessageDefinition_() { return "int64 formationIndex\n\
//\n\
//\n\
//"; }
//public:
//  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }
//
//  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }
//
//  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
//  {
//    ros::serialization::OStream stream(write_ptr, 1000000000);
//    ros::serialization::serialize(stream, formationIndex);
//    return stream.getData();
//  }
//
//  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
//  {
//    ros::serialization::IStream stream(read_ptr, 1000000000);
//    ros::serialization::deserialize(stream, formationIndex);
//    return stream.getData();
//  }
//
//  ROS_DEPRECATED virtual uint32_t serializationLength() const
//  {
//    uint32_t size = 0;
//    size += ros::serialization::serializationLength(formationIndex);
//    return size;
//  }
//
//  typedef boost::shared_ptr< ::Simulator::FormationIndexResponse_<ContainerAllocator> > Ptr;
//  typedef boost::shared_ptr< ::Simulator::FormationIndexResponse_<ContainerAllocator>  const> ConstPtr;
//  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
//}; // struct FormationIndexResponse
//typedef  ::Simulator::FormationIndexResponse_<std::allocator<void> > FormationIndexResponse;
//
//typedef boost::shared_ptr< ::Simulator::FormationIndexResponse> FormationIndexResponsePtr;
//typedef boost::shared_ptr< ::Simulator::FormationIndexResponse const> FormationIndexResponseConstPtr;
//
//struct FormationIndex
//{
//
//typedef FormationIndexRequest Request;
//typedef FormationIndexResponse Response;
//Request request;
//Response response;
//
//typedef Request RequestType;
//typedef Response ResponseType;
//}; // struct FormationIndex
//} // namespace Simulator
//
//namespace ros
//{
//namespace message_traits
//{
//template<class ContainerAllocator> struct IsMessage< ::Simulator::FormationIndexRequest_<ContainerAllocator> > : public TrueType {};
//template<class ContainerAllocator> struct IsMessage< ::Simulator::FormationIndexRequest_<ContainerAllocator>  const> : public TrueType {};
//template<class ContainerAllocator>
//struct MD5Sum< ::Simulator::FormationIndexRequest_<ContainerAllocator> > {
//  static const char* value()
//  {
//    return "d41d8cd98f00b204e9800998ecf8427e";
//  }
//
//  static const char* value(const  ::Simulator::FormationIndexRequest_<ContainerAllocator> &) { return value(); }
//  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
//  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
//};
//
//template<class ContainerAllocator>
//struct DataType< ::Simulator::FormationIndexRequest_<ContainerAllocator> > {
//  static const char* value()
//  {
//    return "Simulator/FormationIndexRequest";
//  }
//
//  static const char* value(const  ::Simulator::FormationIndexRequest_<ContainerAllocator> &) { return value(); }
//};
//
//template<class ContainerAllocator>
//struct Definition< ::Simulator::FormationIndexRequest_<ContainerAllocator> > {
//  static const char* value()
//  {
//    return "\n\
//\n\
//";
//  }
//
//  static const char* value(const  ::Simulator::FormationIndexRequest_<ContainerAllocator> &) { return value(); }
//};
//
//template<class ContainerAllocator> struct IsFixedSize< ::Simulator::FormationIndexRequest_<ContainerAllocator> > : public TrueType {};
//} // namespace message_traits
//} // namespace ros
//
//
//namespace ros
//{
//namespace message_traits
//{
//template<class ContainerAllocator> struct IsMessage< ::Simulator::FormationIndexResponse_<ContainerAllocator> > : public TrueType {};
//template<class ContainerAllocator> struct IsMessage< ::Simulator::FormationIndexResponse_<ContainerAllocator>  const> : public TrueType {};
//template<class ContainerAllocator>
//struct MD5Sum< ::Simulator::FormationIndexResponse_<ContainerAllocator> > {
//  static const char* value()
//  {
//    return "3e0302d241547a13e5cebd81ed40641e";
//  }
//
//  static const char* value(const  ::Simulator::FormationIndexResponse_<ContainerAllocator> &) { return value(); }
//  static const uint64_t static_value1 = 0x3e0302d241547a13ULL;
//  static const uint64_t static_value2 = 0xe5cebd81ed40641eULL;
//};
//
//template<class ContainerAllocator>
//struct DataType< ::Simulator::FormationIndexResponse_<ContainerAllocator> > {
//  static const char* value()
//  {
//    return "Simulator/FormationIndexResponse";
//  }
//
//  static const char* value(const  ::Simulator::FormationIndexResponse_<ContainerAllocator> &) { return value(); }
//};
//
//template<class ContainerAllocator>
//struct Definition< ::Simulator::FormationIndexResponse_<ContainerAllocator> > {
//  static const char* value()
//  {
//    return "int64 formationIndex\n\
//\n\
//\n\
//";
//  }
//
//  static const char* value(const  ::Simulator::FormationIndexResponse_<ContainerAllocator> &) { return value(); }
//};
//
//template<class ContainerAllocator> struct IsFixedSize< ::Simulator::FormationIndexResponse_<ContainerAllocator> > : public TrueType {};
//} // namespace message_traits
//} // namespace ros
//
//namespace ros
//{
//namespace serialization
//{
//
//template<class ContainerAllocator> struct Serializer< ::Simulator::FormationIndexRequest_<ContainerAllocator> >
//{
//  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
//  {
//  }
//
//  ROS_DECLARE_ALLINONE_SERIALIZER;
//}; // struct FormationIndexRequest_
//} // namespace serialization
//} // namespace ros
//
//
//namespace ros
//{
//namespace serialization
//{
//
//template<class ContainerAllocator> struct Serializer< ::Simulator::FormationIndexResponse_<ContainerAllocator> >
//{
//  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
//  {
//    stream.next(m.formationIndex);
//  }
//
//  ROS_DECLARE_ALLINONE_SERIALIZER;
//}; // struct FormationIndexResponse_
//} // namespace serialization
//} // namespace ros
//
//namespace ros
//{
//namespace service_traits
//{
//template<>
//struct MD5Sum<Simulator::FormationIndex> {
//  static const char* value()
//  {
//    return "3e0302d241547a13e5cebd81ed40641e";
//  }
//
//  static const char* value(const Simulator::FormationIndex&) { return value(); }
//};
//
//template<>
//struct DataType<Simulator::FormationIndex> {
//  static const char* value()
//  {
//    return "Simulator/FormationIndex";
//  }
//
//  static const char* value(const Simulator::FormationIndex&) { return value(); }
//};
//
//template<class ContainerAllocator>
//struct MD5Sum<Simulator::FormationIndexRequest_<ContainerAllocator> > {
//  static const char* value()
//  {
//    return "3e0302d241547a13e5cebd81ed40641e";
//  }
//
//  static const char* value(const Simulator::FormationIndexRequest_<ContainerAllocator> &) { return value(); }
//};
//
//template<class ContainerAllocator>
//struct DataType<Simulator::FormationIndexRequest_<ContainerAllocator> > {
//  static const char* value()
//  {
//    return "Simulator/FormationIndex";
//  }
//
//  static const char* value(const Simulator::FormationIndexRequest_<ContainerAllocator> &) { return value(); }
//};
//
//template<class ContainerAllocator>
//struct MD5Sum<Simulator::FormationIndexResponse_<ContainerAllocator> > {
//  static const char* value()
//  {
//    return "3e0302d241547a13e5cebd81ed40641e";
//  }
//
//  static const char* value(const Simulator::FormationIndexResponse_<ContainerAllocator> &) { return value(); }
//};
//
//template<class ContainerAllocator>
//struct DataType<Simulator::FormationIndexResponse_<ContainerAllocator> > {
//  static const char* value()
//  {
//    return "Simulator/FormationIndex";
//  }
//
//  static const char* value(const Simulator::FormationIndexResponse_<ContainerAllocator> &) { return value(); }
//};
//
//} // namespace service_traits
//} // namespace ros
//
//#endif // SIMULATOR_SERVICE_FORMATIONINDEX_H
//
