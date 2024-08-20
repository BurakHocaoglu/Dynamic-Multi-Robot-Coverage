cmake_minimum_required(VERSION 3.1...3.18)
project(coverage_control2)

#add_compile_options(-std=c++17 -Wall -fext-numeric-literals)
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
find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)

if(CGAL_Qt5_FOUND)
  add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)
endif()

add_message_files(
   FILES
   AgentState.msg
   AgentStateArray.msg
   Polygon.msg
   HistoryStep.msg
   PolygonWithHoles.msg
   UtilityDebug.msg
   GeodesicPartition.msg
   WDStep.msg
)

add_service_files(
   FILES
   SetInitialPose.srv
   SetId.srv
   PrintMass.srv
   MultipleTermination.srv
   # GetMetricPartition.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
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

add_executable(dist_coverage_node src/coverage_node.cpp src/coverage_agent.cpp src/coverage_utils.cpp)
target_link_libraries(dist_coverage_node ${catkin_LIBRARIES} CGAL::CGAL CGAL::CGAL_Core CGAL::CGAL_Qt5)
