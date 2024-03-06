cmake_minimum_required(VERSION 2.8.3)
project(fast_mapping)
add_definitions(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    nav_msgs
    geometry_msgs    
    roscpp
    rospy
    std_msgs
    tf
)

find_package(Eigen3 REQUIRED)
#set (EIGEN_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/Eigen3)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  nav_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${jps_search_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fast_mapping_node 
               src/fast_mapping_node.cpp 
               src/fast_mapping.cpp
               src/jps_search.cpp 
               include/fast_mapping/fast_mapping.h
               include/fast_mapping/jps_search.h)
add_dependencies(fast_mapping_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fast_mapping_node ${catkin_LIBRARIES})
