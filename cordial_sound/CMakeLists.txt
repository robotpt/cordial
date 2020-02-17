cmake_minimum_required(VERSION 2.8.3)
project(cordial_sound)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  Sound.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY scripts src srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
