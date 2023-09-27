cmake_minimum_required(VERSION 3.0.2)
project(teleop_listener)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  
)

catkin_package()


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(listener listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_executable(purepursuit purepursuit.cpp)
target_link_libraries(purepursuit ${catkin_LIBRARIES})



