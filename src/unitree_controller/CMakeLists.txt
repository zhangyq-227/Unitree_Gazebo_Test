cmake_minimum_required(VERSION 3.13)
project(unitree_controller)

# add_compile_options(-std=c++14)

set(CMAKE_CXX_FLAGS "-O3 -fPIC -std=c++17")

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
    std_msgs
    tf
    geometry_msgs
    unitree_legged_msgs
)

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
    CATKIN_DEPENDS 
    unitree_legged_msgs 
)

set(ONNX_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/onnx)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${ONNX_ROOT_DIR}/include
)


link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Declare a C++ library
add_library(${PROJECT_NAME}
    src/body.cpp 
)

add_dependencies(${PROJECT_NAME} unitree_legged_msgs_gencpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${EXTRA_LIBS} pthread
)

# add_library(unitreeFootContactPlugin SHARED plugin/foot_contact_plugin.cc)
# target_link_libraries(unitreeFootContactPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# add_library(unitreeDrawForcePlugin SHARED plugin/draw_force_plugin.cc)
# target_link_libraries(unitreeDrawForcePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(unitree_external_force src/external_force.cpp)
target_link_libraries(unitree_external_force ${catkin_LIBRARIES})

add_executable(unitree_servo src/servo.cpp)
target_link_directories(unitree_servo PRIVATE "${ONNX_ROOT_DIR}/lib")
target_link_libraries(unitree_servo ${PROJECT_NAME} ${catkin_LIBRARIES} onnxruntime)

add_executable(unitree_move_kinetic src/move_publisher.cpp)
target_link_libraries(unitree_move_kinetic ${catkin_LIBRARIES} ${EXTRA_LIBS})

add_executable(keyboard_ctrl src/keyboard.cpp)
target_link_libraries(keyboard_ctrl ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(keyboard_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS scripts/vision.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
