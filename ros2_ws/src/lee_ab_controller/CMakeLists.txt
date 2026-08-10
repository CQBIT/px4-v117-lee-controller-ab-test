cmake_minimum_required(VERSION 3.8)
project(lee_ab_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(lee_ab_controller src/lee_ab_controller.cpp)
target_compile_features(lee_ab_controller PUBLIC cxx_std_17)
target_link_libraries(lee_ab_controller Eigen3::Eigen)
ament_target_dependencies(lee_ab_controller rclcpp px4_msgs)

install(TARGETS lee_ab_controller DESTINATION lib/${PROJECT_NAME})
ament_package()
