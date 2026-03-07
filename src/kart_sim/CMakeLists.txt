cmake_minimum_required(VERSION 3.8)
project(kart_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS
  scripts/cone_follower_node.py
  scripts/perfect_perception_node.py
  scripts/capture_frames.py
  scripts/evaluate_perception.py
  scripts/camera_info_fix_node.py
  scripts/ign_cmd_relay.py
  scripts/ackermann_to_vel.py
  scripts/esp32_sim_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
