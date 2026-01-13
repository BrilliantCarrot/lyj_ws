cmake_minimum_required(VERSION 3.8)
project(uav_gnc)

# 1) C++ 표준 지정(C++17로)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2) 의존성, find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 3) 워크스페이스 루트 경로(패키지가 uav_gnc_ws(워크스페이스 최상단)/src/uav_gnc 일 때)
set(WS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(WS_INCLUDE_DIR ${WS_ROOT}/include)
set(WS_CORE_DIR ${WS_ROOT}/core)

message(STATUS "WS_ROOT = ${WS_ROOT}")
message(STATUS "WS_INCLUDE_DIR = ${WS_INCLUDE_DIR}")
message(STATUS "WS_CORE_DIR = ${WS_CORE_DIR}")

# 4) core sources 들
set(CORE_SOURCES
  ${WS_CORE_DIR}/dynamics/sixdof.cpp
  ${WS_CORE_DIR}/control/controller.cpp
)

# 5) core 라이브러리 타겟 생성
add_library(uav_gnc_core ${CORE_SOURCES})
target_include_directories(uav_gnc_core PUBLIC
  ${WS_INCLUDE_DIR}
)

# core 라이브러리에도 컴파일 옵션 추가 가능(TODO)
# target_compile_options(uav_gnc_core PRIVATE -O2)

# 6) 노드 실행파일 추가 (네가 만든 노드 파일명에 맞게)
add_executable(simulation_node src/simulation_node.cpp)
add_executable(control_node src/control_node.cpp)
add_executable(guidance_node src/guidance_node.cpp)
add_executable(navigation_node src/navigation_node.cpp)

# 7) 노드들이 core 라이브러리 링크
target_link_libraries(simulation_node uav_gnc_core)
target_link_libraries(control_node uav_gnc_core)
target_link_libraries(guidance_node uav_gnc_core)
target_link_libraries(navigation_node uav_gnc_core)

# 8) 노드 타겟에 ROS2 의존성 연결
ament_target_dependencies(simulation_node
  rclcpp geometry_msgs nav_msgs sensor_msgs
)
ament_target_dependencies(control_node
  rclcpp geometry_msgs nav_msgs sensor_msgs
)
ament_target_dependencies(guidance_node
  rclcpp geometry_msgs nav_msgs sensor_msgs
)
ament_target_dependencies(navigation_node
  rclcpp geometry_msgs nav_msgs sensor_msgs
)

# 9) 설치(install) 규칙: ros2 run/launch가 되도록
install(TARGETS
  simulation_node
  control_node
  guidance_node
  navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

# 10) launch/config 설치(패키지 공유 경로에 복사)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
