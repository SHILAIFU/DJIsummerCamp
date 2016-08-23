cmake_minimum_required(VERSION 2.8.3)
project(uav_vision)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  message_generation
  std_msgs
)

add_message_files(
	FILES
	DetectInfo.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES moveDetec
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

# Find ourselves lib
include_directories("/usr/local/include/")
file(GLOB aruco_LIBRARIES /usr/local/lib/*.so)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(detectArucoMarker src/trackMarker.cxx )
#target_link_libraries(detectArucoMarker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${aruco_LIBRARIES})

#add_executable(detecMarker src/detectMarker.cxx )
#target_link_libraries(detecMarker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(detecSquareUp src/detectSquareUp.cxx )
#target_link_libraries(detecSquareUp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(detecLed src/ledDetect.cpp)
#target_link_libraries(detecLed ${catkin_LIBRARIES} ${OpenCV_LIBS}
#)

add_executable(detecLedBox src/detecLedBox.cxx)
target_link_libraries(detecLedBox ${catkin_LIBRARIES} ${OpenCV_LIBS}
    ${aruco_LIBRARIES}
)
