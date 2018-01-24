# Copyright 2015, Giacomo Dabisias"
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# @Author 
# Giacomo Dabisias, PhD Student
# PERCRO, (Laboratory of Perceptual Robotics)
# Scuola Superiore Sant'Anna
# via Luigi Alamanni 13D, San Giuliano Terme 56010 (PI), Italy

cmake_minimum_required(VERSION 2.8)
Project(kinect2grabber CXX)

set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE  Release)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/modules")
set(INCLUDE           "${CMAKE_SOURCE_DIR}/include")

option(WITH_OPENCL "adds opencl support for freenect2" OFF)
if(${WITH_OPENCL})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_OPENCL")
endif()

option(WITH_CUDA "adds cuda support for freenect2" OFF)
if(${WITH_CUDA})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_CUDA")
endif()

option(WITH_SERIALIZATION "adds support for boost serialization" OFF)
if(${WITH_SERIALIZATION})
	find_package(Boost REQUIRED)
	include_directories(${INCLUDE})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_SERIALIZATION")
	set(BOOST_LIBS "boost_serialization")
endif()

option(WITH_ROS "adds support for ROS" OFF)
if(${WITH_ROS})
	find_package(catkin REQUIRED COMPONENTS   roscpp std_msgs pcl_conversions  pcl_ros
						sensor_msgs cv_bridge image_transport tf
						message_generation )
	catkin_package( CATKIN_DEPENDS roscpp rospy std_msgs pcl_conversions  pcl_ros
						sensor_msgs cv_bridge image_transport tf
						cmake_modules
	)
	include_directories(${catkin_INCLUDE_DIRS} include)
endif()

option(WITH_PCL "adds pcl cloud support" ON)
if(${WITH_PCL})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_PCL")
	find_package(PCL REQUIRED)
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})
	include_directories(${PCL_INCLUDE_DIRS})
endif()

find_package(OpenCV     REQUIRED)
find_package(freenect2  REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${freenect2_INCLUDE_DIRS})
include_directories(${INCLUDE})

add_executable(Kinect2Grabber      test.cpp)
add_executable(MultiKinect2Grabber multi_test.cpp)
add_executable(pcl_pcd_recorder pcl_pcd_recorder.cpp)
add_executable(pcl_pcd_multiCamrecorder pcl_pcd_multiCamrecorder.cpp)
add_executable(pcl_pcd_multiCamrecorderThreads pcl_pcd_multiCamrecorderThreads.cpp)
#add_executable(pcl_ply_multiCamrecorderThreads pcl_ply_multiCamrecorderThreads.cpp)

add_executable(pcl_ply_multiCamrecorderThreadsnew pcl_ply_multiCamrecorderThreadsnew.cpp)
add_executable(pcl_ply_multiCamrecorderThreadsRValue pcl_ply_multiCamrecorderThreadsRValue.cpp)
add_executable(pcl_ply_multiCamrecorderThreadsConstRef pcl_ply_multiCamrecorderThreadsConstRef.cpp)
add_executable(pcl_video_player pcl_video_player.cpp)
add_executable(pcl_vtk_player pcl_vtk_player.cpp)


add_executable(pairwise_incr_reg  pairwise_incr_reg.cpp)
add_executable(registration_vis  registration_vis.cpp)
add_executable(multi_grabb_registr  multi_grabb_registr.cpp)
add_executable(multi_grabb_registr2  multi_grabb_registr2.cpp)

add_executable(multiGrabberAndRegister  multiGrabberAndRegister.cpp)
add_executable(multiGrabberAndPairwiseRegister  multiGrabberAndPairwiseRegister.cpp)
add_executable(multiGrabberAndReadTransforms  multiGrabberAndReadTransforms.cpp)
add_executable(multiGrabberAndReadTransformsThreads  multiGrabberAndReadTransformsThreads.cpp)


add_executable(multiGrabberAndReadTransformsConcatenate  multiGrabberAndReadTransformsConcatenate.cpp)
add_executable(multiGrabberAndReadTransformsConcatenateThreads  multiGrabberAndReadTransformsConcatenateThreads.cpp)




add_executable(mesh  mesh.cpp)
add_executable(mesh_mls  mesh_mls.cpp)
add_executable(mls_smoothing mls_smoothing.cpp)
add_executable(mls_smoothing_copy mls_smoothing_copy.cpp)

	
add_executable(poisson_recon  poisson_recon.cpp)
add_executable(marching_cube  marching_cube.cpp)
add_executable(poisson_recontruction poisson_recontruction.cpp)

add_executable(ndt  ndt.cpp)
add_executable(example1  example1.cpp)
add_executable(example2  example2.cpp)
add_executable(bspline_fitting bspline_fitting.cpp)
add_executable(staticticalRemoval staticticalRemoval.cpp)
add_executable(radiusOutlierRemoval radiusOutlierRemoval.cpp)


target_link_libraries(Kinect2Grabber      ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(MultiKinect2Grabber ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(pcl_pcd_recorder ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(pcl_pcd_multiCamrecorder ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(pcl_pcd_multiCamrecorderThreads  ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})

target_link_libraries(pcl_ply_multiCamrecorderThreadsnew  ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})

target_link_libraries(pcl_ply_multiCamrecorderThreadsRValue  ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(pcl_ply_multiCamrecorderThreadsConstRef  ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})

target_link_libraries(pcl_video_player  ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})

target_link_libraries(pairwise_incr_reg  ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(multi_grabb_registr  ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(multi_grabb_registr2  ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})

target_link_libraries(ndt  ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(registration_vis  ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(example1  ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})

target_link_libraries(example2  ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})

target_link_libraries(multiGrabberAndRegister  ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})

target_link_libraries(multiGrabberAndPairwiseRegister ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(multiGrabberAndReadTransforms ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(multiGrabberAndReadTransformsThreads  ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(multiGrabberAndReadTransformsConcatenate ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(multiGrabberAndReadTransformsConcatenateThreads ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})

target_link_libraries(mesh ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(mesh_mls ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(mls_smoothing ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(mls_smoothing_copy ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})


target_link_libraries(pcl_vtk_player ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(poisson_recon ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(marching_cube ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(poisson_recontruction ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})

target_link_libraries(bspline_fitting ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(staticticalRemoval ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(radiusOutlierRemoval  ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})

if(${WITH_ROS})
	add_executable(RosKinect2Grabber test_ros.cpp)
	target_link_libraries(RosKinect2Grabber ${catkin_LIBRARIES} ${OpenCV_LIBS} ${FREENECT2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
endif()

