cmake_minimum_required(VERSION 2.8.3)
project(mrl_laser_stabilizer)


find_package(catkin REQUIRED COMPONENTS roscpp qt_build mrl_common_protocol)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtNetwork)

catkin_package(
 INCLUDE_DIRS
  include
 LIBRARIES
 CATKIN_DEPENDS
  roscpp qt_build mrl_common_protocol
 DEPENDS
  Qt4 


)

include_directories(include)
include(${QT_USE_FILE})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${QT_LIBRARIES})

file(GLOB_RECURSE QT_USB_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/mrl_laser_stabilizer/abstract_dynamixels_controller.hpp
include/mrl_laser_stabilizer/usb_dynamixels_controller.hpp
include/mrl_laser_stabilizer/usb_stabilizer_controller.hpp)
file(GLOB_RECURSE QT_UDP_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/mrl_laser_stabilizer/abstract_dynamixels_controller.hpp
include/mrl_laser_stabilizer/udp_dynamixels_controller.hpp
include/mrl_laser_stabilizer/udp_stabilizer_controller.hpp
include/mrl_laser_stabilizer/socket.hpp
)
file(GLOB_RECURSE QT_ARDUINO_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/mrl_laser_stabilizer/abstract_dynamixels_controller.hpp
include/mrl_laser_stabilizer/arduino_dynamixels_controller.hpp
include/mrl_laser_stabilizer/arduino_stabilizer_controller.hpp
include/mrl_laser_stabilizer/socket.hpp
)



QT4_WRAP_CPP(QT_USB_MOC_HPP ${QT_USB_MOC})
QT4_WRAP_CPP(QT_UDP_MOC_HPP ${QT_UDP_MOC})
QT4_WRAP_CPP(QT_ARDUINO_MOC_HPP ${QT_ARDUINO_MOC})


file(GLOB_RECURSE QT_USB_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/abstract_dynamixels_controller.cpp src/usb_dynamixels_controller.cpp src/usb_stabilizer_controller.cpp)
file(GLOB_RECURSE QT_UDP_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/abstract_dynamixels_controller.cpp src/udp_dynamixels_controller.cpp src/udp_stabilizer_controller.cpp src/socket.cpp)
file(GLOB_RECURSE QT_ARDUINO_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/abstract_dynamixels_controller.cpp src/arduino_dynamixels_controller.cpp src/arduino_stabilizer_controller.cpp src/socket.cpp)


##############################################################################
# Binaries
##############################################################################

add_executable(usb_laser_stabilizer ${QT_USB_SOURCES} ${QT_USB_MOC_HPP})
add_executable(udp_laser_stabilizer ${QT_UDP_SOURCES} ${QT_UDP_MOC_HPP})
add_executable(arduino_laser_stabilizer ${QT_ARDUINO_SOURCES} ${QT_ARDUINO_MOC_HPP})


target_link_libraries(usb_laser_stabilizer ${QT_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(udp_laser_stabilizer ${QT_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(arduino_laser_stabilizer ${QT_LIBRARIES} ${catkin_LIBRARIES})

