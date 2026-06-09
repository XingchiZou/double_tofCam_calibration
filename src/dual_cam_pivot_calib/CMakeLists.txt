cmake_minimum_required(VERSION 3.0.2)
project(dual_cam_pivot_calib)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  std_srvs
)

find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(dual_cam_pivot_calib_node src/dual_cam_pivot_calib_node.cpp)
target_link_libraries(dual_cam_pivot_calib_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(dual_cam_pivot_calib_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS dual_cam_pivot_calib_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

catkin_install_python(PROGRAMS scripts/synthetic_pivot_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
