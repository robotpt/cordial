cmake_minimum_required(VERSION 2.8.3)
project(cordial_gui)

find_package(catkin REQUIRED
  message_generation
  rospy
)

add_message_files(
  FILES
  Content.msg
)
add_service_files(
  FILES
  Prompt.srv
)
generate_messages()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
