cmake_minimum_required(VERSION 3.8)
project(test_ground)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

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

add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs std_srvs)

add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber rclcpp std_msgs std_srvs)

add_executable(custom_publisher src/custom_publisher.cpp)
ament_target_dependencies(custom_publisher rclcpp std_msgs std_srvs)

install(TARGETS
    simple_publisher
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS
    simple_subscriber
    DESTINATION lib/${PROJECT_NAME})    

install(TARGETS
custom_publisher
    DESTINATION lib/${PROJECT_NAME})    
ament_package()
