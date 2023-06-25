# Install script for directory: /home/yibeim/devel/Touch_ROS_Driver-master/catin_ws/src/Geomagic_Touch_ROS_Drivers/robotiq_ft_sensor

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/yibeim/devel/Touch_ROS_Driver-master/catin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_ft_sensor/msg" TYPE FILE FILES "/home/yibeim/devel/Touch_ROS_Driver-master/catin_ws/src/Geomagic_Touch_ROS_Drivers/robotiq_ft_sensor/msg/ft_sensor.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_ft_sensor/cmake" TYPE FILE FILES "/home/yibeim/devel/Touch_ROS_Driver-master/catin_ws/build/Geomagic_Touch_ROS_Drivers/robotiq_ft_sensor/catkin_generated/installspace/robotiq_ft_sensor-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/yibeim/devel/Touch_ROS_Driver-master/catin_ws/devel/include/robotiq_ft_sensor")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/yibeim/devel/Touch_ROS_Driver-master/catin_ws/devel/share/roseus/ros/robotiq_ft_sensor")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/yibeim/devel/Touch_ROS_Driver-master/catin_ws/devel/share/common-lisp/ros/robotiq_ft_sensor")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/yibeim/devel/Touch_ROS_Driver-master/catin_ws/devel/share/gennodejs/ros/robotiq_ft_sensor")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/yibeim/devel/Touch_ROS_Driver-master/catin_ws/devel/lib/python3/dist-packages/robotiq_ft_sensor")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/yibeim/devel/Touch_ROS_Driver-master/catin_ws/devel/lib/python3/dist-packages/robotiq_ft_sensor")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/yibeim/devel/Touch_ROS_Driver-master/catin_ws/build/Geomagic_Touch_ROS_Drivers/robotiq_ft_sensor/catkin_generated/installspace/robotiq_ft_sensor.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_ft_sensor/cmake" TYPE FILE FILES "/home/yibeim/devel/Touch_ROS_Driver-master/catin_ws/build/Geomagic_Touch_ROS_Drivers/robotiq_ft_sensor/catkin_generated/installspace/robotiq_ft_sensor-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_ft_sensor/cmake" TYPE FILE FILES
    "/home/yibeim/devel/Touch_ROS_Driver-master/catin_ws/build/Geomagic_Touch_ROS_Drivers/robotiq_ft_sensor/catkin_generated/installspace/robotiq_ft_sensorConfig.cmake"
    "/home/yibeim/devel/Touch_ROS_Driver-master/catin_ws/build/Geomagic_Touch_ROS_Drivers/robotiq_ft_sensor/catkin_generated/installspace/robotiq_ft_sensorConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_ft_sensor" TYPE FILE FILES "/home/yibeim/devel/Touch_ROS_Driver-master/catin_ws/src/Geomagic_Touch_ROS_Drivers/robotiq_ft_sensor/package.xml")
endif()

