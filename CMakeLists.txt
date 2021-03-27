cmake_minimum_required(VERSION 2.8.3)
project(robot_faces)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    roscpp
    roslib
    # rospy
    std_msgs
    message_generation
)


set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake
)

find_package(SFML REQUIRED)


generate_dynamic_reconfigure_options(
    cfg/Parameters.cfg
)


catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES robot_faces
    CATKIN_DEPENDS dynamic_reconfigure roscpp roslib std_msgs message_runtime
    #  DEPENDS system_lib
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${SFML_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node 
    src/robot_faces_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
)



target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${SFML_LIBRARIES}
)
