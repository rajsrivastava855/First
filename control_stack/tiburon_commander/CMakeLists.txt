cmake_minimum_required(VERSION 2.8.3)
project(tiburon_commander)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

###################################
## catkin specific configuration ##
###################################

catkin_package()

##############
## Qt Files ##
##############
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
set(QT_MOC
    include/tiburon_commander/commander_window.h
    include/tiburon_commander/process_widget.h
    include/tiburon_commander/output_widget.h
)
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
SET(QT_SOURCES
  src/commander_window.cpp
  src/process_widget.cpp
  src/output_widget.cpp
)

###########
## Build ##
###########

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS}
    ${QT_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)

add_library(${PROJECT_NAME}_ui
  ${QT_SOURCES}
  ${QT_FORMS_HPP}
  ${QT_MOC_HPP}
)

target_link_libraries(${PROJECT_NAME}_ui ${QT_LIBRARIES})

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_ui
)
