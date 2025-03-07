cmake_minimum_required(VERSION 3.8)
project(pointcloud_converter)

# 默认使用C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})

# 创建可执行文件
add_executable(converter_node 
  src/converter.cpp
)

# 链接依赖库
target_link_libraries(converter_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)

# 安装目标
install(TARGETS converter_node
  DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖项
ament_export_dependencies(
  rclcpp
  sensor_msgs
)

ament_package()