cmake_minimum_required(VERSION 3.8)
project(my_robot_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Bağımlılıkları bul
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 1. Gün Düğümleri: Publisher ve Subscriber
add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs)

add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber rclcpp std_msgs)

# 4. Gün Düğümü: Otonom Engelden Kaçma
add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
ament_target_dependencies(obstacle_avoidance rclcpp sensor_msgs geometry_msgs)

# Çalıştırılabilir dosyaları ROS 2 sistemine kur
install(TARGETS
  simple_publisher
  simple_subscriber
  obstacle_avoidance
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()