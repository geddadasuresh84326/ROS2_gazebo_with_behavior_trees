cmake_minimum_required(VERSION 3.8)
project(behavior_tree_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)

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

add_executable(simple_bt src/simple_bt.cpp)
add_executable(black_board_ports src/black_board_ports.cpp)
<<<<<<< HEAD
add_executable(ports_with_generic_types src/ports_with_generic_types.cpp)
add_executable(reactive_sequence src/reactive_sequence.cpp)
ament_target_dependencies(simple_bt behaviortree_cpp rclcpp)
ament_target_dependencies(black_board_ports behaviortree_cpp rclcpp)
ament_target_dependencies(ports_with_generic_types behaviortree_cpp rclcpp)
ament_target_dependencies(reactive_sequence behaviortree_cpp rclcpp)

install(TARGETS simple_bt DESTINATION lib/${PROJECT_NAME})
install(TARGETS black_board_ports DESTINATION lib/${PROJECT_NAME})
install(TARGETS ports_with_generic_types DESTINATION lib/${PROJECT_NAME})
install(TARGETS reactive_sequence DESTINATION lib/${PROJECT_NAME})
=======
ament_target_dependencies(simple_bt behaviortree_cpp rclcpp)
ament_target_dependencies(black_board_ports behaviortree_cpp rclcpp)

install(TARGETS simple_bt DESTINATION lib/${PROJECT_NAME})
install(TARGETS black_board_ports DESTINATION lib/${PROJECT_NAME})
>>>>>>> 7a315364c6bcd56bb4e0c2f039ab0d67971ca904

install(DIRECTORY bt_xml/ DESTINATION share/${PROJECT_NAME}/bt_xml)
ament_package()
