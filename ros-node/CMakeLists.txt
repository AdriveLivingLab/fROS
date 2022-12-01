cmake_minimum_required(VERSION 3.0.2)
project(FRos)

add_compile_options(-std=c++17 -Wno-packed-bitfield-compat)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    rospy
    std_msgs
)

find_package(Boost REQUIRED COMPONENTS system date_time)

add_message_files(
  FILES
  Frame_1.msg
)
 generate_messages(
  DEPENDENCIES
  std_msgs
 )

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  std_msgs
  DEPENDS
  Boost
  message_runtime
)

set(use_SYSTEM_BOOST TRUE)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM {Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_core
  include/ixxat_gw/decode.h
  include/ixxat_gw/convert.h
  src/ixxat_gw.cpp
)

add_executable(
  ${PROJECT_NAME} src/ixxat_gw.cpp
 )

add_dependencies(${PROJECT_NAME}_core
${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  )

  install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
