cmake_minimum_required(VERSION 3.8)
project(asr_summer_school)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(frontier_detection
  src/frontier_detection.cpp
)
target_include_directories(frontier_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(frontier_detection
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
)
set_target_properties(frontier_detection PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(frontier_detection_node SHARED
  src/frontier_detection_node.cpp
)
target_link_libraries(frontier_detection_node frontier_detection)
ament_target_dependencies(frontier_detection_node rclcpp rclcpp_components nav_msgs geometry_msgs visualization_msgs)
rclcpp_components_register_node(frontier_detection_node
  PLUGIN "FrontierDetectionNode"
  EXECUTABLE frontier_detection_node_exe
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/example_nav_to_pose.py
  ${PROJECT_NAME}/tag_landmarks.py
  ${PROJECT_NAME}/motion_guard.py
  ${PROJECT_NAME}/mission_orchestrator.py
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS frontier_detection frontier_detection_node
  EXPORT export_frontier_detection
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config test worlds models
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME mission_policy
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/test -p test_mission_*.py -v
  )
  add_test(NAME mission_offline_regressions
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/test -p test_offline_regressions.py -v
  )
  set_tests_properties(mission_policy mission_offline_regressions PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}"
  )

  # test_guard_ros.py matched neither pattern above, so the one test that
  # exercises the real /cmd_vel gate never ran.  It drives actual ROS
  # transport, so pin it to the isolated localhost domain 87 and to a
  # middleware that needs no router: it must not reach the robot's domain 8
  # nor the simulation's 116, whatever the shell happens to export.
  add_test(NAME guard_transport
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/test -p test_guard_ros.py -v
  )
  set_tests_properties(guard_transport PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH};ROS_DOMAIN_ID=87;ROS_LOCALHOST_ONLY=1;RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
    TIMEOUT 120
  )
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
