cmake_minimum_required(VERSION 3.8)
project(drone_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(el_force REQUIRED)

add_executable(attacker src/attacker.cpp)
add_executable(defender src/defender.cpp)

ament_target_dependencies(attacker rclcpp geometry_msgs el_force)
ament_target_dependencies(defender rclcpp geometry_msgs el_force)

target_include_directories(attacker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(defender PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(attacker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(defender PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  attacker
  "geometry_msgs"
)
ament_target_dependencies(
  defender
  "geometry_msgs"
)

install(TARGETS attacker
  DESTINATION lib/${PROJECT_NAME}
  )
install(TARGETS defender
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
