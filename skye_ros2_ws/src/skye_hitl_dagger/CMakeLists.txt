cmake_minimum_required(VERSION 3.8)
project(skye_hitl_dagger)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PolicyActionChunk.msg"
  "msg/ControlMode.msg"
  DEPENDENCIES std_msgs
)
# rosidl_generator_py already installs the skye_hitl_dagger Python package (msg bindings).
# Install application Python modules into the same package namespace.
ament_get_python_install_dir(python_install_dir)
install(DIRECTORY skye_hitl_dagger/
  DESTINATION "${python_install_dir}/skye_hitl_dagger"
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/control_arbiter
  DESTINATION lib/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
