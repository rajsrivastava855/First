cmake_minimum_required(VERSION 2.8.3)
project(thruster_controller)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  hammerhead
  qt_build
  roscpp
  roslib
)

find_package(Qt4 REQUIRED COMPONENTS
  QtCore
  QtGui
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(FILES
  ThrusterSpeeds.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

################################################
## Find files
################################################
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
set(QT_MOC include/thruster_controller/thruster_control.h)
SET(QT_SOURCES src/thruster_control.cpp)

################################################
## Setup QT
################################################
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include(
  ${QT_USE_FILE}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${QT_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_library(thruster_control ${QT_SOURCES} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(thruster_control ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(thruster_control ${PROJECT_NAME}_generate_messages_cpp)

add_executable(thruster_controller src/thruster_controller.cpp)
target_link_libraries(thruster_controller thruster_control)
