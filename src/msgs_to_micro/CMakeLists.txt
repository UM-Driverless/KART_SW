cmake_minimum_required(VERSION 3.8)
project(msgs_to_micro)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  ackermann_msgs
)
include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/msgs_to_micro/comms_micro.cpp

)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(comms_micro src/main_comms.cpp)
ament_target_dependencies(comms_micro ${dependencies})
target_link_libraries(comms_micro ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  comms_micro
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY DESTINATION share/${PROJECT_NAME})

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
