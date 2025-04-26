cmake_minimum_required(VERSION 3.0.2)
project(my_ros_project)

# 设置 C++ 编译标准
set(CMAKE_CXX_STANDARD 17)

# 找到依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  geometry_msgs
)

# 包含头文件
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

# 声明生成库
add_executable(point_projector_node src/point_projector.cpp)

# 链接库
target_link_libraries(point_projector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# 安装目标
install(TARGETS point_projector_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
