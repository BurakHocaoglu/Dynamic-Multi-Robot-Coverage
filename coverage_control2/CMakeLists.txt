cmake_minimum_required(VERSION 3.1...3.18)
project(coverage_control2)

#add_compile_options(-std=gnu++11 -fext-numeric-literals)
add_compile_options(-std=c++17 -fext-numeric-literals)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  std_srvs
  message_generation
  message_runtime
)

set(CMAKE_BUILD_TYPE "Release")
# set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

find_package(CGAL REQUIRED COMPONENTS Core)
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
   FILES
   AgentState.msg
   AgentStateArray.msg
   Polygon.msg
   HistoryStep.msg
   PolygonWithHoles.msg
)

add_service_files(
   FILES
   SetInitialPose.srv
   SetId.srv
   PrintMass.srv
)

generate_messages(
   DEPENDENCIES
   geometry_msgs  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES coverage_control2
  CATKIN_DEPENDS gazebo_msgs geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/coverage_control2.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(dist_coverage_node src/coverage_node.cpp src/coverage_agent.cpp src/coverage_utils.cpp)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(dist_coverage_node ${catkin_LIBRARIES} CGAL::CGAL CGAL::CGAL_Core)
