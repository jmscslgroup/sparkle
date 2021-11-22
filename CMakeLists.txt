cmake_minimum_required(VERSION 2.8.3)
project(sparkle)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  gazebo_ros_control
  geometry_msgs
  gazebo_msgs
  position_controllers
  roscpp
  rospy
  sensor_msgs
  sicktoolbox
  sicktoolbox_wrapper
  std_msgs
  tf
  transmission_interface
  velocity_controllers
  velodyne_pointcloud
)


set(ROS_FOUND FALSE)
if(DEFINED ENV{ROS_DISTRO})
  set(FOUND_ROS_DISTRO $ENV{ROS_DISTRO})
  set(ROS_FOUND TRUE)
else()
  message("ROS distro variable not set. Trying to figure it out...")
  set(AVAILABLE_ROS_VERSIONS "noetic;melodic;lunar;kinetic;jade;indigo")
  set(ROS_FOUND FALSE)
  foreach(version ${AVAILABLE_ROS_VERSIONS})
    if(NOT ROS_FOUND)
      find_path(ROS_H ros.h PATHS /opt/ros/${version}/include/ros)
      if(ROS_H)
        message("Found ros version ${version}")
        set(FOUND_ROS_DISTRO ${version})
        set(ROS_FOUND TRUE)
      endif()
    endif()
  endforeach()
endif()

if(ROS_FOUND)
  if($ENV{ROS_DISTRO} STREQUAL "noetic")
	  message("Using ROS Noetic")
    # Do stuff specific to Kinetic
    find_package(gazebo 11 REQUIRED)
    #find_package(ignition-msgs6 REQUIRED)
  endif()
else()
  message("ROS distro is unknown.")
endif()


## System dependencies are found with CMake's conventions

find_package(Boost REQUIRED COMPONENTS system)

# Need to have these in order to find/include boost stuff
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS}
# ${catkin_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES catvehicle
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs velodyne_pointcloud tf
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include include/sparkle)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(
   /usr/include/eigen3
 )
## Declare a C++ library
add_library(sparkle_odom SHARED src/odom.cc)

## Specify libraries to link a library or executable target against
target_link_libraries(sparkle_odom ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Declare a C++ library
add_library(min_distance SHARED src/distance.cc)

## Specify libraries to link a library or executable target against
target_link_libraries(min_distance ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable( cmd_sparkle src/cmd_sparkle.cc )
target_link_libraries( cmd_sparkle ${catkin_LIBRARIES} )

add_executable( model_state src/model_state.cc )
target_link_libraries( model_state ${catkin_LIBRARIES} )

add_executable( states_wnode src/states_wnode.cpp )
target_link_libraries( states_wnode ${catkin_LIBRARIES} )

add_executable( sclock src/sclock.cpp )
target_link_libraries( sclock ${catkin_LIBRARIES} )

add_executable( pause src/pause.cpp )
target_link_libraries( pause ${catkin_LIBRARIES} )

add_executable( bicycle src/bicycle.cpp )
target_link_libraries( bicycle ${catkin_LIBRARIES} )

add_executable( transform_sparkle src/transform.cpp )
target_link_libraries( transform_sparkle ${catkin_LIBRARIES} )

add_executable( companion_timer src/companion_timer.cpp )
target_link_libraries(companion_timer ${catkin_LIBRARIES} )

add_executable( micromodel 
	src/controller.cc
	src/micromodel_controller.cc )
target_link_libraries(micromodel ${catkin_LIBRARIES} )

add_executable( relativevel src/relvel.cpp )
target_link_libraries( relativevel ${catkin_LIBRARIES} )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )




#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# added for running the simulation details without need to have 
# the information locally sourced
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Mark executables and/or libraries for installation
#install(TARGETS sparklemodel
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sparkle.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
