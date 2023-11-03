cmake_minimum_required(VERSION 2.8.3)
project(pandora_fusion)

set(${PROJECT_NAME}_CATKIN_DEPS
    angles
    nodelet
    pcl_ros
    roscpp
    roslib
    sensor_msgs
    eigen_conversions)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
include_directories(include ${catkin_INCLUDE_DIRS})


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Boost REQUIRED)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -fopenmp")

catkin_package()

######################
#       fusion       #
######################
add_executable(fusion_node src/fusion_node.cc src/fusion.cc)
target_link_libraries(fusion_node
	${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

install(
    TARGETS
    fusion_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
