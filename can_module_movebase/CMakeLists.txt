##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(can_module_movebase)
add_compile_options(-std=c++11)

##############################################################################
# Catkin
##############################################################################
find_package(yaml-cpp)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  autoware_msgs
)



set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

#find_package(Qt5  REQUIRED COMPONENTS  Widgets  )
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
  include
  include/can_module
  ${catkin_INCLUDE_DIRS}
)

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS rviz roscpp)
#include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
  ${catkin_LIBRARY_DIRS}
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

#catkin_install_python( PROGRAMS
#  scripts/mpc_ctrl.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#
#)

##############################################################################
# Qt Environment
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
#find_package(catkin REQUIRED COMPONENTS rviz roscpp)

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

#MESSAGE("QT_LIBRARIES")
#include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything.


##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/can_module/*.hpp *.h)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)


# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

#file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
#file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
#file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/can_gui/*.hpp)

#QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
#QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
#QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

#file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(can_module_movebase src/pure_pursuit.cpp src/pid.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(can_module_movebase yaml-cpp ${QT_LIBRARIES} ${catkin_LIBRARIES} libcontrolcan.so -lpthread)
install(TARGETS can_module_movebase RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
