cmake_minimum_required(VERSION 3.8)
project(localiztion_evaluator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib")

# 查找 ROS2 组件
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gps_msg REQUIRED)  # 你的自定义消息包
find_package(GeographicLib REQUIRED )
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# 生成可执行文件
add_executable(localiztion_evaluator_node src/position_comparator.cpp)



# 链接库
ament_target_dependencies(localiztion_evaluator_node
  rclcpp
  geometry_msgs
  gps_msg  # 你的自定义消息
  nav_msgs
  GeographicLib
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 安装可执行文件
install(TARGETS localiztion_evaluator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
