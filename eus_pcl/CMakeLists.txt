cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

rosbuild_find_ros_package(euslisp)
include_directories(/usr/include /usr/X11R6/include ${euslisp_PACKAGE_PATH}/jskeus/eus/include)

rosbuild_add_library(eus_pcl src/euspcl_main.cpp src/euspcl_common.cpp src/euspcl_features.cpp src/euspcl_filters.cpp src/euspcl_io.cpp src/euspcl_octree.cpp src/euspcl_range_image.cpp src/euspcl_registration.cpp src/euspcl_sample_consensus.cpp src/euspcl_segmentation.cpp src/euspcl_surface.cpp src/euspcl_tracking.cpp src/euspcl_recognition.cpp)

# compile flags
set(COMPILE_FLAGS "-fPIC -O2 -Wno-write-strings -Wno-comment")
add_definitions(-DLinux -D_REENTRANT -DVERSION='\"${8.26}\"' -DTHREADED -DPTHREAD -DX11R6_1)
add_definitions(-DSVNVERSION='\"r${SVNVERSION}\"')
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64* OR
   ${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64* )
 add_definitions(-Dx86_64)
else()
 add_definitions(-Di486)
endif()
add_definitions(-D__PCL_SELECT=17)
rosbuild_add_compile_flags(eus_pcl ${COMPILE_FLAGS})

set_target_properties(eus_pcl PROPERTIES PREFIX "" SUFFIX ".so")
