cmake_minimum_required(VERSION 2.8.3)
project(rtsp_cam)

#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -O2 -fopenmp -mavx2 -fPIC")
set(FFmpeg_INC /usr/local/ffmpeg/include)
set(FFmpeg_LIB /usr/local/ffmpeg/lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${FFmpeg_INC}
)

link_libraries(${FFmpeg_LIB}/libavdevice.so)
link_libraries(${FFmpeg_LIB}/libavfilter.so)
link_libraries(${FFmpeg_LIB}/libavformat.so)
link_libraries(${FFmpeg_LIB}/libavutil.so)
link_libraries(${FFmpeg_LIB}/libswresample.so)
link_libraries(${FFmpeg_LIB}/libswscale.so)
link_libraries(${FFmpeg_LIB}/libavcodec.so)

#add_library(${PROJECT_NAME}_nodelet src/send_image.cpp src/transdata/transdata.cpp)
#target_link_libraries(${PROJECT_NAME}_nodelet PRIVATE ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(${PROJECT_NAME}_node send_image.cpp transdata/transdata.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
