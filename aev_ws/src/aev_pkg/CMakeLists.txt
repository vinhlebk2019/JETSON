cmake_minimum_required(VERSION 3.0.2)
project(aev_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(OpenCV_DIR /usr/local/lib/cmake/opencv4)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  OpenCV REQUIRED
  #realsense2
  serial
  roscpp
  rospy
  std_msgs
  message_generation
)

#find_package(realsense2 2.48.0)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

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
add_message_files(
  FILES
  driving_mode_msg.msg
  ecu_feedback_msg.msg
  gui_msg.msg
  lane_detection_msg.msg
  mpc_msg.msg
  object_detection_msg.msg
  radar_msg.msg
  system_monitor_msg.msg
)

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
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  LIBRARIES aev_pkg
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS system_lib
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

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/aev_pkg.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/aev_pkg_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_aev_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

set(DEPENDENCIES realsense2 ${OpenCV_LIBS})

add_executable(talker src/test/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/test/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(test_all src/test/test_all.cpp)
target_link_libraries(test_all ${catkin_LIBRARIES})

include_directories ("/usr/local/include/opencv4")
add_executable(ObjectDetection 
src/ObjectDetection/main.cpp 
src/ObjectDetection/depth_handler.cpp 
src/ObjectDetection/Graphic.cpp 
src/ObjectDetection/rgb_image_handler.cpp 
src/ObjectDetection/rotation_estimator.cpp 
src/ObjectDetection/SSDModel.cpp 
src/ObjectDetection/main.h 
src/ObjectDetection/depth_handler.h
src/ObjectDetection/Graphic.h 
src/ObjectDetection/imu_handler.h 
src/ObjectDetection/MessageQueue.h 
src/ObjectDetection/rgb_image_handler.h 
src/ObjectDetection/rotation_estimator.h 
src/ObjectDetection/SSDModel.h
)
target_link_libraries(ObjectDetection ${catkin_LIBRARIES} ${OpenCV_LIBS} realsense2)

add_executable(DrivingModeSel 
src/DrivingModeSel/DrivingModeSel_main.cpp
src/DrivingModeSel/DrivingModeSel.cpp
src/DrivingModeSel/DrivingModeSel.h
)
target_link_libraries(DrivingModeSel ${catkin_LIBRARIES})

add_executable(SystemMonitor 
src/SystemMonitor/SystemMonitor_main.cpp
src/SystemMonitor/SystemMonitor.cpp
src/SystemMonitor/SystemMonitor.h
)
target_link_libraries(SystemMonitor ${catkin_LIBRARIES})

add_executable(Radar 
src/Radar/ttcRadar.cpp
src/Radar/ttcRadar_Cfg.cpp
)
target_link_libraries(Radar ${catkin_LIBRARIES})

add_executable(MPC 
src/MPC/ert_main.cpp
src/MPC/GenCode_MPC.cpp
src/MPC/GenCode_MPC_data.cpp
src/MPC/rtGetInf.cpp
src/MPC/rtGetNaN.cpp
src/MPC/rt_nonfinite.cpp
src/MPC/GenCode_MPC.h
src/MPC/GenCode_MPC_private.h
src/MPC/GenCode_MPC_types.h
src/MPC/rt_assert.h
src/MPC/rtGetInf.h
src/MPC/rtGetNaN.h
src/MPC/rt_nonfinite.h
src/MPC/rtwtypes.h
src/MPC/rtw_continuous.h
src/MPC/rtw_solver.h
)
target_link_libraries(MPC ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS 
src/CAN/CAN_Node.py
src/CAN/support_class.py
src/CAN/talker_py.py
src/CAN/listener_py.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS 
src/LaneDetection/Lane_Detection.py
src/LaneDetection/full_CNN_model.h5
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


