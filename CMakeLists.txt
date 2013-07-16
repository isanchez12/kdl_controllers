cmake_minimum_required(VERSION 2.8.3)
project(kdl_controllers)

find_package(catkin REQUIRED COMPONENTS controllers_msgs realtime_tools
  control_toolbox urdf controller_interface forward_command_controller)

find_package(Eigen REQUIRED)
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

add_library( ${PROJECT_NAME}
  src/inverse_dynamics_controller.cpp include/controllers/inverse_dynamics_controller.h
  )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS control_toolbox forward_command_controller controllers_msgs urdf controller_interface realtime_tools 
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )


# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

