cmake_minimum_required(VERSION 2.8.3)
project(cordial_manager)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  rospy
)

catkin_python_setup()

add_service_files(
  FILES
  SetString.srv
)

add_action_files(
  FILES
  AskOnGui.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package(CATKIN_DEPENDS
  actionlib_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY scripts src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
