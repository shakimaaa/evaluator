cmake_minimum_required(VERSION 3.8)
project(octomap_evaluator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# 找到所需的依赖包
find_package(rclcpp REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(evaluator_node src/evaluator_node.cpp)

# 链接库
ament_target_dependencies(evaluator_node
  rclcpp
  octomap
  octomap_msgs
  geometry_msgs
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

install(TARGETS evaluator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
