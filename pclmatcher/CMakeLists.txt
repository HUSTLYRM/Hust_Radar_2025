cmake_minimum_required(VERSION 3.0.2)
project(pclmatcher)
set(CMAKE_BUILD_TYPE "Debug")

set(PYTHON_EXECUTABLE "/home/nvidia/miniconda3/envs/Radar/bin/python3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  geometry_msgs
)

add_subdirectory(csf)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(SRC
    src/pclmatcher.cpp 
    src/mycsf.cpp
    src/fasteuclideancluster.cpp
    src/euclideancluster.cpp
)



add_executable(${PROJECT_NAME}_node ${SRC})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  CSF
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
