# OPTAR Library

This is a C++ implementation of OPTAR code that compiles into a library.
It is setup with GNU Autotools and is designed to be built for mobile as well as desktop. Right now (as of January 2020) only Android platform has been tested and verified.

**Clone repo with submodules:**

```
git clone https://github.com/peetonn/optarLibrary --recursive
```

## Prerequisites

* [docopt](https://github.com/docopt/docopt.cpp) (git submodule)
* [spdlog](https://github.com/gabime/spdlog) (git submodule)
* OpenCV
* [roscpp](http://wiki.ros.org/roscpp)

## Build prerequisites

Before building optar library, prerequisites must be setup or built for the target platform. Depending on the platform, this process might get quite involved.

### Android (OUTDATED)

For building library for the Android, one need to setup [Android ndk](https://developer.android.com/ndk).

#### OpenCV

Get the latest version of OpenCV from [here](https://sourceforge.net/projects/opencvlibrary/files/opencv-android/) and extract archive into `thirdparty` folder

#### roscpp

The process of building roscpp for Android is a lengthy one and following [official tutorial](http://wiki.ros.org/android_ndk) does not help, as it is very outdated.
The most recent Docker setup for roscpp that worked for this project dated April 2019 can be found [here](https://github.com/Intermodalics/ros_android).

Follow these steps for roscpp Android crosscompilation (you'll need [Docker](https://www.docker.com/)):

```
git clone https://github.com/Intermodalics/ros_android
cd ros_android
docker/build.sh
docker/run.sh
./install.sh out
```

The build process may take couple hours to complete.
Once finished, `out` folder will contains compiled binaries of static libs that need to be packaged into one static archive.
To do that, navigate to `optarLibrary/thirdparty` and follow these steps:

```
cd <optarLibrary path>
cd thirdparty
mkdir -p roscpp/roscpp_android_ndk/jni
ln -s <ros_android path>/out/target/include roscpp/roscpp_android_ndk/include
ln -s <ros_android path>/out/target/lib roscpp/roscpp_android_ndk/lib
ln -s <ros_android path>/out/target/share roscpp/roscpp_android_ndk/share
```

Finally, inside `roscpp/roscpp_android_ndk/jni` folder, place this `Android.mk` file:

```
LOCAL_PATH := $(call my-dir)

stlibs := xmlrpcpp Bullet3Geometry  boost_stacktrace_basic  diagnostic_aggregator  pcl_recognition  orocos-bfl  yaml-cpp  bz2  boost_math_tr1  charset  amcl_sensors  base_local_planner  theoraenc  vorbisfile  orocos-kdl  opencv_imgcodecs3  opencv_xobjdetect3  opencv_videoio3  SDLmain  tf2  boost_signals  image_publisher  SDL_image  image_proc  opencv_reg3  xml2  camera_calibration_parsers  move_base  boost_container  joint_state_listener  boost_context  bondcpp  boost_math_c99f  camera_info_manager  opencv_calib3d3  boost_math_c99l  navfn  tinyxml2  pcl_io_ply  boost_iostreams  opencv_xfeatures2d3  opencv_stereo3  urdfdom_world  boost_thread  eigen_conversions  boost_program_options  roslib  boost_coroutine  pcl_common  opencv_xphoto3  PocoNet  boost_timer  Bullet3Dynamics  opencv_ml3  boost_contract  ogg  opencv_plot3  collada-dom2.4-dp  tf  rosbag_storage  opencv_rgbd3  boost_type_erasure  interactive_markers  boost_log_setup  tinyxml  boost_atomic  flann_cpp_s-gd  pcl_search  laser_geometry  boost_random  pcl_ros_surface   boost_date_time  opencv_structured_light3  urdf  theora  opencv_optflow3  params  qhullcpp  uuid  pcl_surface  map_server_image_loader  rosconsole_backend_interface  urdfdom_model  LinearMath  tf2_ros  Bullet3OpenCL_clew  vorbisenc  pcl_features  pluginlib_tutorials  tf_conversions  opencv_fuzzy3  pcl_registration  opencv_saliency3  boost_test_exec_monitor  theoradec  boost_stacktrace_noop   opencv_img_hash3  opencv_ccalib3  boost_system  PocoUtild  opencv_tracking3  opencv_superres3  opencv_core3  lz4  opencv_surface_matching3  pointcloud_filters  roscpp_serialization  opencv_phase_unwrapping3  compressed_image_transport  compressed_depth_image_transport  move_slow_and_clear  PocoXML  assimp  pcl_kdtree  PocoJSON  opencv_aruco3  cpp_common console_bridge rosconsole_bridge pcl_ros_filters  opencv_ximgproc3  pcl_io  opencv_bgsegm3  boost_exception  pcl_sample_consensus  layers  Bullet3Collision  BulletCollision  robot_state_publisher_solver  opencv_imgproc3  depth_image_proc  rosbag  pcl_filters  stereo_image_proc  octomap  pcl_segmentation  opencv_video3  pcl_stereo  rosconsole_android  boost_math_c99  kdl_conversions  boost_prg_exec_monitor  opencv_dnn3  opencv_line_descriptor3  image_transport_plugins  amcl_map  opencv_objdetect3  pcl_octree  polled_camera  boost_math_tr1l  boost_math_tr1f  voxel_grid  flann_cpp_s  qhullstatic_r  actionlib  boost_wave  PocoUtil  opencv_bioinspired3  image_geometry  theora_image_transport  opencv_text3  kdl_parser  urdfdom_sensor  Bullet3Common  pcl_ros_tf  opencv_highgui3  costmap_2d  opencv_dpm3  Bullet2FileLoader  carrot_planner  nodeletlib  BulletSoftBody  pcl_keypoints  pcl_ros_segmentation  curl  opencv_features2d3  increment  mean  PocoXMLd  boost_log  cv_bridge  roscpp  rotate_recovery  opencv_photo3  SDL  pcl_ros_features  clear_costmap_recovery  opencv_datasets3  rospack  random_numbers  boost_graph  BulletDynamics  iconv  image_rotate  dynamic_reconfigure_config_init_mutex  image_transport  opencv_shape3  octomath  amcl_pf  opencv_flann3  nodelet_math  PocoJSONd  pcl_ros_io  median  rostime  boost_regex  trajectory_planner_ros  message_filters  opencv_videostab3  pcl_ml  PocoFoundationd  global_planner  roslz4  resource_retriever  boost_wserialization  rosconsole  pluginlib  boost_unit_test_framework  opencv_face3  octomap_ros  PocoFoundation  transfer_function  qhullstatic  laser_scan_filters  opencv_stitching3  class_loader  vorbis  urdfdom_model_state  boost_filesystem  geometric_shapes  boost_chrono  boost_serialization  PocoNetd  dwa_local_planner  topic_tools

define include_shlib
$(eval include $$(CLEAR_VARS))
$(eval LOCAL_MODULE := $(1))
$(eval LOCAL_SRC_FILES := $$(LOCAL_PATH)/../lib/lib$(1).so)
$(eval include $$(PREBUILT_SHARED_LIBRARY))
endef
define include_stlib
$(eval include $$(CLEAR_VARS))
$(eval LOCAL_MODULE := $(1))
$(eval LOCAL_SRC_FILES := ../lib/lib$(1).a)
$(eval include $$(PREBUILT_STATIC_LIBRARY))
endef

$(foreach stlib,$(stlibs),$(eval $(call include_stlib,$(stlib))))

include $(CLEAR_VARS)
LOCAL_MODULE    := roscpp_android_ndk
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_EXPORT_CPPFLAGS := -fexceptions -frtti
LOCAL_CPP_FEATURES := exceptions
LOCAL_EXPORT_LDLIBS := $(foreach l,$(stlibs),-l$(l)) -L$(LOCAL_PATH)/../lib
LOCAL_EXPORT_LDLIBS += -L$(LOCAL_PATH)/../share/OpenCV-3.3.1-dev/3rdparty/lib -ltegra_hal
LOCAL_STATIC_LIBRARIES := $(stlibs) c++_static

include $(BUILD_STATIC_LIBRARY)
```

Build library as:

```
cd roscpp/roscpp_android_ndk
ndk-build
```

### iOS

Download everything from [here](https://drive.google.com/drive/folders/1vSUwomhf6c1z4rKpnr_vYnVB8_uRuYsx?usp=sharing) into *thirdparty* folder.

### macOS

*TBD*

```
brew install opencv spdlog
```

### Ubuntu

*TBD*

## Build library

### Android

```
cd Android
ndk-build
```

### iOS

1. Generate Xcode project files:

```
mkdir build-ios
cd build-ios
cmake .. -GXcode -DCMAKE_SYSTEM_NAME=iOS
```

2. Open *optar.xcodeproj* and build the library.

### macOS

*TBD*

### Ubuntu

*TBD*
