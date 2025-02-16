cmake_minimum_required(VERSION 3.10)
project(ttmpc_franka)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../rbdl" )
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}" )


INCLUDE(cmake/FindRBDL.cmake)

add_definitions(-DBUILD_DIRECTORY=\"${CMAKE_SOURCE_DIR}\") 
message(STATUS "BUILD_DIRECTORY = ${CMAKE_SOURCE_DIR}")

find_package(catkin REQUIRED COMPONENTS
roscpp
dynamic_reconfigure
franka_hw
franka_gripper
geometry_msgs
nav_msgs
hardware_interface
pluginlib
realtime_tools
roscpp
rospy
roslib
tf2
tf2_ros
)

find_package(Eigen3 REQUIRED)
find_package(RBDL REQUIRED)
find_package(Franka 0.5.0 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found, enabling support")
    add_definitions(-DUSE_OPENMP)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    franka_hw
    nav_msgs
    geometry_msgs
    hardware_interface
    pluginlib
    realtime_tools
    roscpp
    tf2 
    tf2_ros
  DEPENDS Franka
)

###########
## Build ##
###########

include_directories(
  include
  ${RBDL_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
src/ttmpc_controller.cpp
src/types.cpp
src/Model/robot_model.cpp
src/Constraints/SelfCollision/SelfCollisionModel.cpp
src/Constraints/EnvCollision/EnvCollisionModel.cpp
src/Interfaces/osqp_interface.cpp
src/Interfaces/solver_interface.cpp
src/Model/model.cpp
src/Model/integrator.cpp
src/Spline/cubic_spline.cpp
src/Spline/cubic_spline_rot.cpp
src/Spline/arc_length_spline.cpp
src/Params/params.cpp
src/Constraints/constraints.cpp
src/Cost/cost.cpp
src/Constraints/bounds.cpp
src/MPC/mpc.cpp
src/Params/track.cpp
include/config.h
include/Model/robot_model.h
)

add_dependencies(ttmpc_franka
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  ${RBDL_LIBRARY}
  OsqpEigen::OsqpEigen
  Threads::Threads
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()
#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES ${PROJECT_NAME}_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(
  PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)