cmake_minimum_required(VERSION 3.22)
project(so_arm100_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# Install Python scripts
install(DIRECTORY scripts/ 
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)
