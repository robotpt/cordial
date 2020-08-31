cmake_minimum_required(VERSION 3.0.2)
project(cordial_msgs)

find_package(catkin REQUIRED
  actionlib_msgs
  geometry_msgs
  message_generation
  rospy
)

add_message_files(
  FILES
  Display.msg
  FaceRequest.msg
  MouseEvent.msg
  Sound.msg
)

add_service_files(
  FILES
  SetString.srv
)

add_action_files(
  FILES
  Ask.action
  AskOnGui.action
)


generate_messages(DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  actionlib_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)