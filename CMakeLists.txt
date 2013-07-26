cmake_minimum_required(VERSION 2.8.3)
project(kdl_controllers)

find_package(catkin REQUIRED COMPONENTS control_msgs realtime_tools
       urdf controller_interface forward_command_controller
       kdl_urdf_tools pluginlib orocos_kdl roscpp
       sensor_msgs) 
# roscpp hardware_interface pluginlib controller_interface sensor_msgs
find_package(Eigen REQUIRED)
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

add_library( ${PROJECT_NAME}
  src/inverse_dynamics_controller.cpp include/kdl_controllers/inverse_dynamics_controller.h
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS sensor_msgs roscpp orocos_kdl pluginlib kdl_urdf_tools forward_command_controller control_msgs urdf controller_interface realtime_tools 
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

