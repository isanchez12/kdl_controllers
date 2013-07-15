# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
cmake_minimum_required(VERSION 2.8.3)
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
project(kdl_controllers)

# Do setup in case of ros package, If ROS_ROOT is set, it is
# recommended to use RTT/OCL through the ros packages.
#
find_package(catkin REQUIRED COMPONENTS hardware_interface kdl_urdf_tools controller_manager
  control_toolbox rospy urdf controller_interface)

# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
#----find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# Add xenomai flags

add_library(${PROJECT_NAME}
  src/controllers/inverse_dynamics_controller.cpp include/controllers/inverse_dynamics_controller.h
  )

## Generate added messages and services with any dependencies listed here

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


#-----------------------------------------------------------

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
#-----include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libcontrollerman_controllers-<target>.so
# and installs in the directory lib/orocos/controllerman_controllers/
#-----orocos_component(controllerman_controllers_components
#-----  src/inverse_dynamics_controller.cpp
#  ) # ...you may add multiple source files
# You may add multiple orocos_component statements.

# Additional headers:
# Installs in the include/orocos/controllerman_controllers/ directory
#----orocos_install_headers(
#----- include/irk_controllers/controllers/trivial.h
#  ) # ...you may add multiple header files
# You may add multiple orocos_install_headers statements.

# Building a Plugin
# Creates a plugin library libcontrollerman_controllers-plugin-<target>.so
# and installs in the directory lib/orocos/controllerman_controllers/plugins/
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
##orocos_plugin(controllerman_controllers-plugin controllerman_controllers-plugin.cpp) # ...only one plugin function per library !
# You may add multiple orocos_plugin statements.


# Building a Service:
# Creates a plugin library libcontrollerman_controllers-service-<target>.so
# and installs in the directory lib/orocos/controllerman_controllers/plugins/
##orocos_service(controllerman_controllers-service controllerman_controllers-service.cpp) # ...only one service per library !
# You may add multiple orocos_service statements.


# Building a typekit (recommended):
# Creates a typekit library libcontrollerman_controllers-types-<target>.so
# and installs in the directory lib/orocos/controllerman_controllers/types/
##orocos_typegen_headers(controllerman_controllers-types.hpp) # ...you may add multiple header files
# You may only have *ONE* orocos_typegen_headers statement !


# Building a normal library (optional):
# Creates a library libsupport-<target>.so and installs it in
# lib/
##orocos_library(support support.cpp) # ...you may add multiple source files
# You may add multiple orocos_library statements.


# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#------orocos_generate_package()




