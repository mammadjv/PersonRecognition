cmake_minimum_required(VERSION 2.8.3)
project(openni_tracker)

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  std_msgs
  image_transport
  kdl_conversions
  openni2_camera
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  tf
  genmsg
  message_generation
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

#rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#uncomment if you have defined messages
#uncomment if you have defined services
#rosbuild_gensrv()

add_message_files(
   FILES
   TrackerUserArray.msg
   TrackerUser.msg
)
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
#  LIBRARIES openni2-tracker
  CATKIN_DEPENDS cv_bridge geometry_msgs std_msgs image_transport kdl_conversions openni2_camera pcl_conversions pcl_ros roscpp roslib tf message_runtime
  DEPENDS system_lib opencv
)



#generate_messages()

#rosbuild_genmsg()


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
# Change two lines below according to your installation

set(OPENNI2_DIR /home/mohammad/catkin_ws/personRecognitionFiles/OpenNI2)
set(OPENNI2_WRAPPER /opt/ros/indigo/include/openni2_camera)
set(NITE2_DIR /home/mohammad/catkin_ws/personRecognitionFiles/NiTE-Linux-x64-2.2)
set(NITE2_LIB /home/mohammad/catkin_ws/personRecognitionFiles/NiTE-Linux-x64-2.2/Redist/libNiTE2.so)

link_directories(${OPENNI2_DIR}/Bin/x64-Release)

include_directories(${OPENNI2_DIR}/Bin/x64-Release)
include_directories(/usr/include/openni2/)
include_directories(${OPENNI2_DIR}/Include)
include_directories(${OPENNI2_WRAPPER})
include_directories(${NITE2_DIR}/Include)
include_directories(${OpenCV_INCLUDE_DIRS}/include)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

link_directories(${OPENNI2_DIR}/Bin/x64-Release)
include_directories(${OPENNI2_DIR}/Include)
link_directories(${NITE2_DIR}/Redist)
include_directories(${NITE2_DIR}/Include)

add_executable(tracker src/tracker.cpp)
target_link_libraries(tracker OpenNI2 ${NITE2_LIB} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})


#add_executable(kinect_tracker src/kinect_tracker.cpp)
#target_link_libraries(kinect_tracker OpenNI2 ${NITE2_LIB} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

add_dependencies(
    tracker
    DEPS ${${PROJECT_NAME}_EXPORTED_TARGETS}}
)
