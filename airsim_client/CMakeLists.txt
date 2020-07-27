cmake_minimum_required(VERSION 3.10.0)
project(airsim_client)

set(AIRSIM_ROOT "/home/henry/AirSim")
set(AIRLIB_ROOT "/home/henry/AirSim/AirLib")
set(CMAKE_CXX_STANDARD 11)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
# add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)
# TODO seem to be necessary, dont know why

set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc  
-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel") 


# set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB_INCLUDES " ${AIRLIB_ROOT}/deps/rpclib/include")
set(MAVLINKCOM_LIB_INCLUDES " ${AIRLIB_ROOT}/deps/MavLinkCom/include")


set(RPC_LIB rpc) # name of .a file with lib prefix
set(MAVLINKCOM_LIB MavLinkCom)
set(AirLib_LIB AirLib)


find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  mavros_msgs
  message_generation
  nav_msgs
  # nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)


add_message_files(
  FILES
  FlatTarget.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES airsim_ros
  CATKIN_DEPENDS message_runtime roscpp std_msgs # nodelet 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${AIRLIB_ROOT}/deps/eigen3
  ${AIRLIB_ROOT}/include
  ${RPC_LIB_INCLUDES}
  ${MAVLINKCOM_LIB_INCLUDES}
  # ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  "/home/henry/AirSim/AirLib/lib"
  "/home/henry/AirSim/AirLib/deps/rpclib/lib"
  "/home/henry/AirSim/AirLib/deps/MavLinkCom/lib"
)


add_executable(airsim_client_node 
  src/airsim_client_node.cpp
  src/airsim_ros_client.cpp
)

add_dependencies(${catkin_EXPORTED_TARGETS})
target_link_libraries(airsim_client_node rpc MavLinkCom AirLib ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})


add_executable(image_listener_node 
  src/image_listener_node.cpp
)


target_link_libraries(image_listener_node  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS image_listener_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(cheat_node 
  src/cheat_node.cpp
)
target_link_libraries(cheat_node  rpc MavLinkCom AirLib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(controller_node 
  src/controller_node.cpp
  src/airsim_ros_client.cpp
)

target_link_libraries(controller_node  ${catkin_LIBRARIES} rpc MavLinkCom AirLib  ${OpenCV_LIBRARIES})

install(TARGETS controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(
  simple_trajectory_node
  src/simple_trajectory_node.cpp
  src/simple_trajectory.cpp
)

install(TARGETS simple_trajectory_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

target_link_libraries(simple_trajectory_node  ${catkin_LIBRARIES})

install(TARGETS cheat_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
