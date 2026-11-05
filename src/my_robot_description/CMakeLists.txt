cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  urdf
  worlds
  config
  maps
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(FILES robot_view.rviz DESTINATION share/${PROJECT_NAME})

ament_package()
