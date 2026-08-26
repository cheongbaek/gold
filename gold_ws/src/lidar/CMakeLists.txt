cmake_minimum_required(VERSION 3.16)
project(lidar LANGUAGES CXX)

# ---------------------------------------------------------------------------
# ★★ 빌드 규약 — 이 패키지에 --symlink-install 을 쓰지 않는다 (2026-08-25) ★★
#   C++ 이 들어 있어서, 심볼릭으로 설치하면 install/lidar/lib/lidar/* 가
#   build/lidar/* 를 가리키는 링크가 된다. 그 상태로 build/ 를 지우면
#   install/ 이 ★깨진 링크만 남은 채★ 남아 "빌드는 됐는데 실행이 이상하게
#   실패한다"가 된다. 실행파일은 실제 파일로 복사되어야 한다.
#
#       colcon build --packages-select lidar \
#           --cmake-args -DCMAKE_BUILD_TYPE=Release
#
#   ※ 이미 심볼릭으로 빌드해 둔 것이 있으면 한 번은 지우고 시작한다:
#       rm -rf build/lidar install/lidar
# ---------------------------------------------------------------------------

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# ROS 2 / third-party
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)

# ---------------------------------------------------------------------------
# cone_lidar_node — 가상범퍼 AEB (차를 움직이지 않는다)
# ---------------------------------------------------------------------------
add_executable(cone_lidar_node src/cone_lidar_node.cpp)
target_include_directories(cone_lidar_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(cone_lidar_node
  rclcpp sensor_msgs visualization_msgs std_msgs geometry_msgs pcl_conversions)
target_link_libraries(cone_lidar_node ${PCL_LIBRARIES})

# ---------------------------------------------------------------------------
# drive_lidar_node — 측량 잠금 + IMU 헤딩홀드 직진
# ---------------------------------------------------------------------------
add_executable(drive_lidar_node src/drive_lidar_node.cpp)
target_include_directories(drive_lidar_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(drive_lidar_node
  rclcpp sensor_msgs visualization_msgs std_msgs geometry_msgs nav_msgs
  pcl_conversions)
target_link_libraries(drive_lidar_node ${PCL_LIBRARIES})

# ---------------------------------------------------------------------------
# pedal_drive_node — ★페달 주행 담당 + AEB★ (one_launch.py 의 구동 담당 노드)
#   PCL 을 쓰지 않는다 — 포인트클라우드를 보지 않고 cone_lidar_node 의 판정
#   (Bool/Float32)과 /encoder(Int32)만 받아 /aeb_stop 하나를 낸다. 의존이 가볍다.
#   ※ geometry_msgs 는 이 노드가 Twist 를 쓰기 때문이 아니라 kasa_units.hpp 가
#     그것을 포함하기 때문이다(그 헤더가 액추에이터 계약 전체를 소유한다).
#     이 노드는 여전히 /cmd_vel_raw 를 발행하지 않는다 — 헤더의 '발행하지 않는 것'.
# ---------------------------------------------------------------------------
add_executable(pedal_drive_node src/pedal_drive_node.cpp)
target_include_directories(pedal_drive_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(pedal_drive_node rclcpp std_msgs geometry_msgs)

# ---------------------------------------------------------------------------
# drive_gps_node — GPS 일자 매핑 + 스탠리 추종
#   ※ white1 driving.py 와 기능이 겹친다. 동시에 띄우지 말 것(README 참고).
# ---------------------------------------------------------------------------
add_executable(drive_gps_node src/drive_gps_node.cpp)
target_include_directories(drive_gps_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(drive_gps_node
  rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs ament_index_cpp)

install(TARGETS
  cone_lidar_node
  drive_lidar_node
  drive_gps_node
  pedal_drive_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)

# ---------------------------------------------------------------------------
# 단위시험 — ★환산 계층이 제일 중요하다★
#   펄스·pot·부호가 틀리면 빌드도 되고 토픽도 붙는데 차만 다르게 움직인다.
#   그 조용한 실패를 책상에서 잡는 것이 이 시험의 목적이다.
# ---------------------------------------------------------------------------
add_executable(test_kasa_units test/test_kasa_units.cpp)
target_include_directories(test_kasa_units PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(test_kasa_units rclcpp geometry_msgs std_msgs)
install(TARGETS test_kasa_units DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_test QUIET)
  if(ament_cmake_test_FOUND)
    ament_add_test(test_kasa_units_run
      COMMAND $<TARGET_FILE:test_kasa_units>
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    )
  endif()
endif()

ament_package()
