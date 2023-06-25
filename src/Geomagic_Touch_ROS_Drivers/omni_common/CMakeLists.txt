cmake_minimum_required(VERSION 2.8.3)
project(omni_common)

find_package(catkin REQUIRED COMPONENTS 
  geometry_msgs 
  omni_msgs 
  sensor_msgs 
  std_msgs 
  tf
  # message_generation
)
# ## Generate messages in the 'msg' folder
# add_message_files(
#   DIRECTORY
#     msg
#   FILES
#     ft_sensor.msg
# )
# ## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
catkin_package(
CATKIN_DEPENDS 
  geometry_msgs 
  omni_description 
  omni_msgs 
  robot_state_publisher 
  sensor_msgs 
  std_msgs 
  tf
  xacro
)

# ###################################
# ## catkin specific configuration ##
# ###################################
# catkin_package(
#   CATKIN_DEPENDS message_runtime std_msgs
# )


include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

find_path(OPENHAPTICS_INCLUDE_DIR NAMES HD/hd.h HL/hl.h HDU/hdu.h)

if(NOT OPENHAPTICS_INCLUDE_DIR)
  set(OPENHAPTICS_MESSAGE "OPENHAPTICS was not found. If you do not have it you will not be able to use the Phantom Omni.")
  message(STATUS "${OPENHAPTICS_MESSAGE}") 
  return()
endif(NOT OPENHAPTICS_INCLUDE_DIR)
  
# add_executable(teleoperation src/teleoperation.cpp)
# add_dependencies(teleoperation ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(teleoperation ${catkin_LIBRARIES})


add_executable(omni src/omni.cpp)
target_link_libraries(omni HD HDU rt ncurses ${catkin_LIBRARIES})
add_dependencies(omni omni_msgs_gencpp)


add_executable(omni_state src/omni_state.cpp)
target_link_libraries(omni_state HD HDU rt ncurses ${catkin_LIBRARIES})
add_dependencies(omni_state omni_msgs_gencpp)

add_executable(teleoperation 
src/kinematics/kinematics_ur.cpp 
src/teleoperation.cpp 
src/QP/QuadProg.cpp 
src/QP/QP_ArmEndeffectorTask.cpp)
add_dependencies(teleoperation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleoperation
   ${catkin_LIBRARIES}
 )

add_executable(teleoperation_test 
src/kinematics/kinematics_ur.cpp 
src/teleoperation_test.cpp 
src/QP/QuadProg.cpp 
src/QP/QP_ArmEndeffectorTask.cpp)
add_dependencies(teleoperation_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleoperation_test
   ${catkin_LIBRARIES}
 )


   add_executable(tcpip_client
   src/teleoperation_tcpip.cpp
   src/kinematics/kinematics_hr.cpp 
   src/QP/QuadProg.cpp 
   src/QP/QP_ArmEndeffectorTask.cpp)
   add_dependencies(tcpip_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
   target_link_libraries(tcpip_client
      ${catkin_LIBRARIES}
    )

    add_executable(ik
    src/ik.cpp
    src/kinematics/kinematics_ur.cpp 
    src/QP/QuadProg.cpp 
    src/QP/QP_ArmEndeffectorTask.cpp)
    add_dependencies(ik ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(ik
       ${catkin_LIBRARIES}
     )
   add_executable(forceex
   src/forceex.cpp
   src/kinematics/kinematics_ur.cpp 
   src/QP/QuadProg.cpp 
   src/QP/QP_ArmEndeffectorTask.cpp)
   add_dependencies(forceex ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
   target_link_libraries(forceex
      ${catkin_LIBRARIES}
    )
 find_package(Boost REQUIRED COMPONENTS unit_test_framework)
 find_package(eigen-quadprog QUIET NO_CMAKE_PACKAGE_REGISTRY)
 target_link_libraries(teleoperation PUBLIC eigen-quadprog::eigen-quadprog)
 target_link_libraries(teleoperation_test PUBLIC eigen-quadprog::eigen-quadprog)
#  target_link_libraries(teleoperation_operability PUBLIC eigen-quadprog::eigen-quadprog)
 target_link_libraries(tcpip_client PUBLIC eigen-quadprog::eigen-quadprog)
 target_link_libraries(ik PUBLIC eigen-quadprog::eigen-quadprog)
 target_link_libraries(forceex PUBLIC eigen-quadprog::eigen-quadprog)
 include_directories(" /home/yibeim/eigen-quadprog/src/")

