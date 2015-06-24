cmake_minimum_required(VERSION 2.8.3)
project(eus_pcl)

# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS euslisp)
find_package(PCL REQUIRED)

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")

find_package(PkgConfig)

# CATKIN_MIGRATION: removed during catkin migration
if(NOT euslisp_INCLUDE_DIRS)
  if(EXISTS ${euslisp_SOURCE_DIR}/jskeus)
    set(euslisp_PACKAGE_PATH ${euslisp_SOURCE_DIR})
  else()
    set(euslisp_PACKAGE_PATH ${euslisp_PREFIX}/share/euslisp)
  endif()
  message("-- Set euslisp_PACKAGE_PATH to ${euslisp_PACKAGE_PATH}")
  set(euslisp_INCLUDE_DIRS ${euslisp_PACKAGE_PATH}/jskeus/eus/include)
endif()
message("-- Set euslisp_INCLUDE_DIRS to ${euslisp_INCLUDE_DIRS}")

include_directories(include /usr/include /usr/X11R6/include ${euslisp_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(eus_pcl SHARED
  src/euspcl_main.cpp
  src/euspcl_common.cpp
  src/euspcl_features.cpp
  src/euspcl_filters.cpp
  src/euspcl_io.cpp
  src/euspcl_octree.cpp
  src/euspcl_range_image.cpp
  src/euspcl_registration.cpp
  src/euspcl_sample_consensus.cpp
  src/euspcl_segmentation.cpp
  src/euspcl_surface.cpp
  src/euspcl_tracking.cpp
  src/euspcl_recognition.cpp)

set_target_properties(eus_pcl PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/euslisp)
set_target_properties(eus_pcl PROPERTIES PREFIX "" SUFFIX ".so")
target_link_libraries(eus_pcl ${PCL_LIBRARIES})

# compile flags
add_definitions(-fPIC -O2 -Wno-write-strings -Wno-comment)
add_definitions(-Di486 -DLinux -D_REENTRANT -DVERSION='\"9.00\"' -DTHREADED -DPTHREAD -DX11R6_1)
add_definitions('-DREPOVERSION="\\"${REPOVERSION}\\""')
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64* OR
   ${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64* )
 add_definitions(-Dx86_64)
else()
 add_definitions(-Di486)
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES Darwin)
 add_definitions(-Dx86_64)
 set(CMAKE_SHARED_LIBRARY_CREATE_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_CREATE_CXX_FLAGS} -flat_namespace -undefined suppress")
endif()

#set_target_properties(eus_pcl PROPERTIES PREFIX "" SUFFIX ".so")

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS pcl
    CATKIN_DEPENDS # euslisp TODO
    INCLUDE_DIRS include # TODO include
    LIBRARIES eus_pcl # TODO
)

# install
#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#install(TARGETS eus_pcl ${eus_pcl_executables}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
