cmake_minimum_required(VERSION 3.16)
project(competition_fsm)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FsmEvent.srv"
)

# Guard against duplicate target creation during CMake re-invocation
if(NOT TARGET ament_cmake_python_build_${PROJECT_NAME}_egg)
  find_package(ament_cmake_python REQUIRED QUIET)
  ament_python_install_package(competition_fsm
    PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}"
  )
endif()

# Install Python source modules alongside rosidl-generated package
install(FILES
  competition_fsm/__init__.py
  competition_fsm/fsm.py
  competition_fsm/fsm_node.py
  DESTINATION local/lib/python3.10/dist-packages/competition_fsm
)

install(PROGRAMS
  scripts/competition_fsm_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
