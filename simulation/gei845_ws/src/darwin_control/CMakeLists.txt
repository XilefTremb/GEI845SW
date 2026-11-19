cmake_minimum_required(VERSION 3.8)
project(darwin_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(Boost REQUIRED thread)

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

add_library(${PROJECT_NAME}_lib SHARED
  src/lib/DarwinJoint.cpp
  src/lib/DarwinJointControl.cpp
  src/lib/DarwinReadFile.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(
    ${PROJECT_NAME}_lib
    rclcpp
    sensor_msgs
    control_msgs
    std_msgs
    Boost
)
install(TARGETS
    ${PROJECT_NAME}_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

add_executable(static_walk src/main.cpp)
target_link_libraries(static_walk PUBLIC 
  rclcpp::rclcpp
  Boost::thread
  ${PROJECT_NAME}_lib
)

add_executable(dynamic_walk src/dynamic_main.cpp)
target_link_libraries(dynamic_walk PUBLIC 
  rclcpp::rclcpp
  Boost::thread
  ${PROJECT_NAME}_lib
)

add_executable(jointSubscriber src/joint_subscriber.cpp)
target_link_libraries(jointSubscriber PUBLIC 
  rclcpp::rclcpp
  Boost::thread
  ${PROJECT_NAME}_lib
)

install(TARGETS
  static_walk
  dynamic_walk
  jointSubscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# --- Make the executables available to the ROS2 ecosystem ---
ament_package()