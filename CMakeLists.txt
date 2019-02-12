cmake_minimum_required(VERSION 2.8.3)
project(bebop2_galago)

find_package(catkin REQUIRED COMPONENTS
  bebop_msgs
  std_msgs
  roscpp
)

catkin_package(  
  INCLUDE_DIRS src
  CATKIN_DEPENDS 
  bebop_msgs
  roscpp 
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(SendStatue src/SendStatue.cpp)
add_executable(PointToPoint src/PointToPoint.cpp)
add_executable(PointToPoint_List src/PointToPoint_List.cpp)

target_link_libraries(SendStatue ${catkin_LIBRARIES})
target_link_libraries(PointToPoint ${catkin_LIBRARIES})
target_link_libraries(PointToPoint_List ${catkin_LIBRARIES})

add_dependencies(SendStatue src/SendStatue.cpp)
add_dependencies(PointToPoint src/PointToPoint.cpp)
add_dependencies(PointToPoint_List src/PointToPoint_List.cpp)
