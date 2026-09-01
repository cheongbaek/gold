cmake_minimum_required(VERSION 3.16)
project(mppi_local_planner LANGUAGES CXX)

# ---------------------------------------------------------------------------
# ★★ 빌드 규약 — 이 패키지에 --symlink-install 을 쓰지 않는다 ★★
#   C++ 이 들어 있어서, 심볼릭으로 설치하면 install/.../lib 가 build/ 를
#   가리키는 링크가 된다. 그 상태로 build/ 를 지우면 install/ 이 깨진다.
#
#       colcon build --packages-select mppi_local_planner \
#           --cmake-args -DCMAKE_BUILD_TYPE=Release
# ---------------------------------------------------------------------------

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(mppi_local_planner_node
  src/mppi_local_planner_node.cpp
  src/ego_costmap.cpp
  src/mppi_controller.cpp
)

ament_target_dependencies(mppi_local_planner_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
)

install(TARGETS
  mppi_local_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
