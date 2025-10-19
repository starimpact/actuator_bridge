cmake_minimum_required(VERSION 3.0.2)
project(actuator_bridge)

# Ensure modern C++ standard for features used (if-with-init, etc.)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  can_msgs
)
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs can_msgs
  INCLUDE_DIRS include
  LIBRARIES actuator_bridge_core actuator_drivers
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(actuator_drivers
  src/drivers/dm_driver.cpp
  src/drivers/robstride_driver.cpp
)
add_dependencies(actuator_drivers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(actuator_drivers ${catkin_LIBRARIES})

# Core bridge library for reuse by other nodes
add_library(actuator_bridge_core
  src/bridge.cpp
)
add_dependencies(actuator_bridge_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(actuator_bridge_core actuator_drivers ${catkin_LIBRARIES})

add_executable(actuator_bridge_node src/actuator_bridge_node.cpp)
add_dependencies(actuator_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(actuator_bridge_node actuator_bridge_core actuator_drivers ${catkin_LIBRARIES})

install(TARGETS actuator_drivers actuator_bridge_core actuator_bridge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/socketcan_spawner.py
  scripts/generate_socketcan_launch.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
