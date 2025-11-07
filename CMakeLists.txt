cmake_minimum_required(VERSION 3.8)
project(project1phase1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(ros2_control REQUIRED)

install(DIRECTORY
urdf
meshes
launch
worlds
config
DESTINATION share/${PROJECT_NAME}/
)
install(
    DIRECTORY include/project1phase1
    DESTINATION include
    )
install(PROGRAMS scripts/keyboard_control.py
        DESTINATION lib/${PROJECT_NAME})

install(
    PROGRAMS
    launch/robot_description_publisher.py
    DESTINATION lib/${PROJECT_NAME}
    )
install(PROGRAMS
  scripts/straight_line_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED) 
  set(ament_cmake_copyright_FOUND TRUE) 
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
