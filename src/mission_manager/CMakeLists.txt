cmake_minimum_required(VERSION 3.16)
project(mission_manager)

find_package(rosidl_default_generators REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToZone.action"
  DEPENDENCIES nav2_msgs action_msgs
)

# Guard against duplicate target creation during CMake re-invocation
if(NOT TARGET ament_cmake_python_build_${PROJECT_NAME}_egg)
  find_package(ament_cmake_python REQUIRED QUIET)
  ament_python_install_package(mission_manager)
endif()

# Install Python source modules alongside rosidl-generated package
install(FILES
  mission_manager/__init__.py
  mission_manager/mission_manager.py
  mission_manager/waypoint_loader.py
  DESTINATION local/lib/python3.10/dist-packages/mission_manager
)

# Install console script
install(PROGRAMS
  scripts/mission_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
