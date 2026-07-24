cmake_minimum_required(VERSION 3.8)
project(moveit_tools_lib)

# C++17 标准（ROS2 Humble 强制）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(safe_thread_queue REQUIRED)
find_package(yaml-cpp REQUIRED)

# 1. 正确：填入src下的cpp实现文件，不要写hpp
add_library(${PROJECT_NAME}
  src/moveit_tools.cpp
)

# 头文件路径对外暴露 PUBLIC，外部包自动继承
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 绑定库依赖 PUBLIC，外部包find_package后自动链接
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
  safe_thread_queue
  yaml-cpp
)

# 2. 关键：安装头文件目录，否则外部包找不到hpp
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装编译后的库文件，并导出目标
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 导出库目标给外部包调用
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
# 导出头文件目录
ament_export_include_directories(include)
# 导出所有依赖，外部包无需重复find_package
ament_export_dependencies(
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
  safe_thread_queue
  yaml-cpp
)

# 测试部分
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
