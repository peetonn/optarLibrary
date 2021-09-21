// Generated by gencpp from file opt_msgs/ArcoreCameraFeatures.msg
// DO NOT EDIT!


#ifndef OPT_MSGS_MESSAGE_ARCORECAMERAFEATURES_H
#define OPT_MSGS_MESSAGE_ARCORECAMERAFEATURES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/CompressedImage.h>
#include "opt_msgs/KeyPoint.h"

namespace opt_msgs
{
template <class ContainerAllocator>
struct ArcoreCameraFeatures_
{
  typedef ArcoreCameraFeatures_<ContainerAllocator> Type;

  ArcoreCameraFeatures_()
    : header()
    , mobileFramePose()
    , image()
    , focal_length_x_px(0.0)
    , focal_length_y_px(0.0)
    , image_width_px(0)
    , image_height_px(0)
    , principal_point_x_px(0.0)
    , principal_point_y_px(0.0)
    , keypoints()
    , descriptors_mat_data()
    , descriptors_mat_cols(0)
    , descriptors_mat_rows(0)
    , descriptors_mat_type(0)
    , deviceId()  {
    }
  ArcoreCameraFeatures_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , mobileFramePose(_alloc)
    , image(_alloc)
    , focal_length_x_px(0.0)
    , focal_length_y_px(0.0)
    , image_width_px(0)
    , image_height_px(0)
    , principal_point_x_px(0.0)
    , principal_point_y_px(0.0)
    , keypoints(_alloc)
    , descriptors_mat_data(_alloc)
    , descriptors_mat_cols(0)
    , descriptors_mat_rows(0)
    , descriptors_mat_type(0)
    , deviceId(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _mobileFramePose_type;
  _mobileFramePose_type mobileFramePose;

   typedef  ::sensor_msgs::CompressedImage_<ContainerAllocator>  _image_type;
  _image_type image;

   typedef double _focal_length_x_px_type;
  _focal_length_x_px_type focal_length_x_px;

   typedef double _focal_length_y_px_type;
  _focal_length_y_px_type focal_length_y_px;

   typedef int32_t _image_width_px_type;
  _image_width_px_type image_width_px;

   typedef int32_t _image_height_px_type;
  _image_height_px_type image_height_px;

   typedef double _principal_point_x_px_type;
  _principal_point_x_px_type principal_point_x_px;

   typedef double _principal_point_y_px_type;
  _principal_point_y_px_type principal_point_y_px;

   typedef std::vector< ::opt_msgs::KeyPoint_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::opt_msgs::KeyPoint_<ContainerAllocator> >::other >  _keypoints_type;
  _keypoints_type keypoints;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _descriptors_mat_data_type;
  _descriptors_mat_data_type descriptors_mat_data;

   typedef int32_t _descriptors_mat_cols_type;
  _descriptors_mat_cols_type descriptors_mat_cols;

   typedef int32_t _descriptors_mat_rows_type;
  _descriptors_mat_rows_type descriptors_mat_rows;

   typedef int32_t _descriptors_mat_type_type;
  _descriptors_mat_type_type descriptors_mat_type;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _deviceId_type;
  _deviceId_type deviceId;





  typedef boost::shared_ptr< ::opt_msgs::ArcoreCameraFeatures_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::opt_msgs::ArcoreCameraFeatures_<ContainerAllocator> const> ConstPtr;

}; // struct ArcoreCameraFeatures_

typedef ::opt_msgs::ArcoreCameraFeatures_<std::allocator<void> > ArcoreCameraFeatures;

typedef boost::shared_ptr< ::opt_msgs::ArcoreCameraFeatures > ArcoreCameraFeaturesPtr;
typedef boost::shared_ptr< ::opt_msgs::ArcoreCameraFeatures const> ArcoreCameraFeaturesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::opt_msgs::ArcoreCameraFeatures_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::opt_msgs::ArcoreCameraFeatures_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace opt_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'opt_msgs': ['/root/workspace/ros/src/open_ptrack/opt_msgs/msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::opt_msgs::ArcoreCameraFeatures_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::opt_msgs::ArcoreCameraFeatures_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::opt_msgs::ArcoreCameraFeatures_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::opt_msgs::ArcoreCameraFeatures_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::opt_msgs::ArcoreCameraFeatures_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::opt_msgs::ArcoreCameraFeatures_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::opt_msgs::ArcoreCameraFeatures_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e89d94590ecb650624dbf1bc14eef650";
  }

  static const char* value(const ::opt_msgs::ArcoreCameraFeatures_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe89d94590ecb6506ULL;
  static const uint64_t static_value2 = 0x24dbf1bc14eef650ULL;
};

template<class ContainerAllocator>
struct DataType< ::opt_msgs::ArcoreCameraFeatures_<ContainerAllocator> >
{
  static const char* value()
  {
    return "opt_msgs/ArcoreCameraFeatures";
  }

  static const char* value(const ::opt_msgs::ArcoreCameraFeatures_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::opt_msgs::ArcoreCameraFeatures_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This message is used to transfer the detected features from ARCore devices\n"
"# together with the pose estimated by ARcore and the camera intrinsic\n"
"# parameters.\n"
"#\n"
"# author: Carlo Rizzardo (c-rizz)\n"
"\n"
"Header header\n"
"\n"
"# The pose of the camera estimated by ARcore\n"
"geometry_msgs/Pose mobileFramePose\n"
"\n"
"# The camera image\n"
"sensor_msgs/CompressedImage image\n"
"\n"
"# The focal lengths in pixels\n"
"float64 focal_length_x_px\n"
"float64 focal_length_y_px\n"
"\n"
"# The image size in pixels\n"
"int32 image_width_px\n"
"int32 image_height_px\n"
"\n"
"# The position of the principal point, in pixels\n"
"float64 principal_point_x_px\n"
"float64 principal_point_y_px\n"
"\n"
"\n"
"opt_msgs/KeyPoint[] keypoints\n"
"uint8[] descriptors_mat_data\n"
"int32 descriptors_mat_cols\n"
"int32 descriptors_mat_rows\n"
"int32 descriptors_mat_type\n"
"\n"
"# Unique identifier of the phone sending the data\n"
"string deviceId\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/CompressedImage\n"
"# This message contains a compressed image\n"
"\n"
"Header header        # Header timestamp should be acquisition time of image\n"
"                     # Header frame_id should be optical frame of camera\n"
"                     # origin of frame should be optical center of camera\n"
"                     # +x should point to the right in the image\n"
"                     # +y should point down in the image\n"
"                     # +z should point into to plane of the image\n"
"\n"
"string format        # Specifies the format of the data\n"
"                     #   Acceptable values:\n"
"                     #     jpeg, png\n"
"uint8[] data         # Compressed image buffer\n"
"\n"
"================================================================================\n"
"MSG: opt_msgs/KeyPoint\n"
"# ROS message used to transfer an OpenCV cv::KeyPoint object\n"
"#\n"
"# author: Carlo Rizzardo (c-rizz)\n"
"\n"
"\n"
"# The y pixel position of the keypoint in the image\n"
"float32 x_pos\n"
"\n"
"# The y pixel position of the keypoint in the image\n"
"float32 y_pos\n"
"\n"
"# The angle of the keypoint\n"
"float32 angle\n"
"\n"
"# object class (if the keypoints need to be clustered by an object they belong to)\n"
"int32 class_id\n"
"\n"
"# octave (pyramid layer) from which the keypoint has been extracted\n"
"int32 octave\n"
"\n"
"# the response by which the most strong keypoints have been selected. Can be used for the further sorting or subsampling\n"
"float32 response\n"
"\n"
"# diameter of the meaningful keypoint neighborhood\n"
"float32 size\n"
;
  }

  static const char* value(const ::opt_msgs::ArcoreCameraFeatures_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::opt_msgs::ArcoreCameraFeatures_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.mobileFramePose);
      stream.next(m.image);
      stream.next(m.focal_length_x_px);
      stream.next(m.focal_length_y_px);
      stream.next(m.image_width_px);
      stream.next(m.image_height_px);
      stream.next(m.principal_point_x_px);
      stream.next(m.principal_point_y_px);
      stream.next(m.keypoints);
      stream.next(m.descriptors_mat_data);
      stream.next(m.descriptors_mat_cols);
      stream.next(m.descriptors_mat_rows);
      stream.next(m.descriptors_mat_type);
      stream.next(m.deviceId);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ArcoreCameraFeatures_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::opt_msgs::ArcoreCameraFeatures_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::opt_msgs::ArcoreCameraFeatures_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "mobileFramePose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.mobileFramePose);
    s << indent << "image: ";
    s << std::endl;
    Printer< ::sensor_msgs::CompressedImage_<ContainerAllocator> >::stream(s, indent + "  ", v.image);
    s << indent << "focal_length_x_px: ";
    Printer<double>::stream(s, indent + "  ", v.focal_length_x_px);
    s << indent << "focal_length_y_px: ";
    Printer<double>::stream(s, indent + "  ", v.focal_length_y_px);
    s << indent << "image_width_px: ";
    Printer<int32_t>::stream(s, indent + "  ", v.image_width_px);
    s << indent << "image_height_px: ";
    Printer<int32_t>::stream(s, indent + "  ", v.image_height_px);
    s << indent << "principal_point_x_px: ";
    Printer<double>::stream(s, indent + "  ", v.principal_point_x_px);
    s << indent << "principal_point_y_px: ";
    Printer<double>::stream(s, indent + "  ", v.principal_point_y_px);
    s << indent << "keypoints[]" << std::endl;
    for (size_t i = 0; i < v.keypoints.size(); ++i)
    {
      s << indent << "  keypoints[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::opt_msgs::KeyPoint_<ContainerAllocator> >::stream(s, indent + "    ", v.keypoints[i]);
    }
    s << indent << "descriptors_mat_data[]" << std::endl;
    for (size_t i = 0; i < v.descriptors_mat_data.size(); ++i)
    {
      s << indent << "  descriptors_mat_data[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.descriptors_mat_data[i]);
    }
    s << indent << "descriptors_mat_cols: ";
    Printer<int32_t>::stream(s, indent + "  ", v.descriptors_mat_cols);
    s << indent << "descriptors_mat_rows: ";
    Printer<int32_t>::stream(s, indent + "  ", v.descriptors_mat_rows);
    s << indent << "descriptors_mat_type: ";
    Printer<int32_t>::stream(s, indent + "  ", v.descriptors_mat_type);
    s << indent << "deviceId: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.deviceId);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPT_MSGS_MESSAGE_ARCORECAMERAFEATURES_H
