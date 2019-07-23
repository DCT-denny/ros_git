################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_bringup)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  OpenCV
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  sensor_msgs
  diagnostic_msgs
  turtlebot3_msgs
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs diagnostic_msgs turtlebot3_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(turtlebot3_diagnostics src/turtlebot3_diagnostics.cpp)
add_dependencies(turtlebot3_diagnostics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlebot3_diagnostics ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS turtlebot3_diagnostics
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS 
  scripts/create_udev_rules
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES 99-turtlebot3-cdc.rules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


add_executable(walkrobot src/walkrobot.cpp)
target_link_libraries(walkrobot ${catkin_LIBRARIES})
add_executable(turn90 src/turn90.cpp)
target_link_libraries(turn90 ${catkin_LIBRARIES})
add_executable(wallandturn src/wallandturn.cpp)
target_link_libraries(wallandturn ${catkin_LIBRARIES})
add_executable(testwallpd src/testwallpd.cpp)
target_link_libraries(testwallpd ${catkin_LIBRARIES})
add_executable(wallcorner src/wallcorner.cpp)
target_link_libraries(wallcorner ${catkin_LIBRARIES})
add_executable(wallcorner2 src/wallcorner2.cpp)
target_link_libraries(wallcorner2 ${catkin_LIBRARIES})
add_executable(walltheta src/walltheta.cpp)
target_link_libraries(walltheta ${catkin_LIBRARIES})
add_executable(twopointparalle src/twopointparalle.cpp)
target_link_libraries(twopointparalle ${catkin_LIBRARIES})
add_executable(laser_msg src/laser_msg.cpp)
target_link_libraries(laser_msg ${catkin_LIBRARIES})
add_executable(lmse src/lmse.cpp)
target_link_libraries(lmse ${catkin_LIBRARIES})
add_executable(ransac src/ransac.cpp)
target_link_libraries(ransac ${catkin_LIBRARIES})
add_executable(testopencv src/testopencv.cpp)
target_link_libraries(testopencv ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(lmse_manypoint src/lmse_manypoint.cpp)
target_link_libraries(lmse_manypoint ${catkin_LIBRARIES})
add_executable(testlmse src/testlmse.cpp)
target_link_libraries(testlmse ${catkin_LIBRARIES})
add_executable(lmse3wall src/lmse3wall.cpp)
target_link_libraries(lmse3wall ${catkin_LIBRARIES})
add_executable(ransacmoveave src/ransacmoveave.cpp)
target_link_libraries(ransacmoveave ${catkin_LIBRARIES})
add_executable(testhough_transform src/testhough_transform.cpp)
target_link_libraries(testhough_transform ${catkin_LIBRARIES})
add_executable(lmse_manyline src/lmse_manyline.cpp)
target_link_libraries(lmse_manyline ${catkin_LIBRARIES})
add_executable(hough_manyline src/hough_manyline.cpp)
target_link_libraries(hough_manyline ${catkin_LIBRARIES})
add_executable(hough_noave src/hough_noave.cpp)
target_link_libraries(hough_noave ${catkin_LIBRARIES})
add_executable(hough_noave_changeaxis src/hough_noave_changeaxis.cpp)
target_link_libraries(hough_noave_changeaxis ${catkin_LIBRARIES})
add_executable(raspberry_hough src/raspberry_hough.cpp)
target_link_libraries(raspberry_hough ${catkin_LIBRARIES})
add_executable(remote_paintwall src/remote_paintwall.cpp)
target_link_libraries(remote_paintwall ${catkin_LIBRARIES})
add_executable(two_error_wall src/two_error_wall.cpp)
target_link_libraries(two_error_wall ${catkin_LIBRARIES})
add_executable(testbreak_urgnode src/testbreak_urgnode.cpp)
target_link_libraries(testbreak_urgnode ${catkin_LIBRARIES})
add_executable(viewlight src/viewlight.cpp)
target_link_libraries(viewlight ${catkin_LIBRARIES})
add_executable(test_funcase_hough src/test_funcase_hough.cpp)
target_link_libraries(test_funcase_hough ${catkin_LIBRARIES})




################################################################################
# Test
################################################################################
