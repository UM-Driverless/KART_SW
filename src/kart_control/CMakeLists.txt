cmake_minimum_required(VERSION 3.8)
project(kart_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS
  scripts/cone_follower_node.py
  scripts/stanley_controller_node.py
  scripts/cmd_vel_bridge_node.py
  scripts/state_machine_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
