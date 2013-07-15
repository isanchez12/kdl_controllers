cmake_minimum_required(VERSION 2.8.3)
project(kdl_controllers)

find_package(catkin REQUIRED COMPONENTS hardware_interface kdl_urdf_tools controller_manager
  control_toolbox rospy urdf controller_interface)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

add_library(${PROJECT_NAME}
  src/inverse_dynamics_controller.cpp include/inverse_dynamics_controller.h
  )

catkin_package(
  DEPENDS hardware_interface urdf irk_simulation irk_model kdl_urdf_tools controller_manager control_toolbox rospy
  CATKIN_DEPENDS # TODO
  INCLUDE_DIRS # TODO include
  LIBRARIES # TODO
  )


# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

