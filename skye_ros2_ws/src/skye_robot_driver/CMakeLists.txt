cmake_minimum_required(VERSION 3.16)
project(skye_robot_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Repo: Skye_ROS_Bridge/third_party/gento_sdk
set(SKYE_BRIDGE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
set(GENTO_SDK_ROOT "${SKYE_BRIDGE_ROOT}/third_party/gento_sdk")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(GENTO_SDK_LIBDIR "${GENTO_SDK_ROOT}/lib/aarch64")
else()
  set(GENTO_SDK_LIBDIR "${GENTO_SDK_ROOT}/lib/x86_64")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetMode.srv"
)

find_library(GENTO_SDK_LIBRARY NAMES GentoSDK
  PATHS "${GENTO_SDK_LIBDIR}" NO_DEFAULT_PATH REQUIRED)

set(GENTO_SDK_INCLUDE_DIRS
  ${GENTO_SDK_ROOT}/include/L1Robot
  ${GENTO_SDK_ROOT}/include/Common
  ${GENTO_SDK_ROOT}/include/L0Control
  ${GENTO_SDK_ROOT}/include/FileClient
  ${GENTO_SDK_ROOT}/include/Kinematics
  ${GENTO_SDK_ROOT}/include/Kinematics/ArmKinematics
  ${GENTO_SDK_ROOT}/include/Kinematics/BaseMath
  ${GENTO_SDK_ROOT}/include/Kinematics/DynaIdent
  ${GENTO_SDK_ROOT}/include/Kinematics/KineCommon
  ${GENTO_SDK_ROOT}/include/Kinematics/MotionPlanner
  ${GENTO_SDK_ROOT}/include/Kinematics/SkyeBodyKinematics)

add_library(skye_driver_core STATIC
  src/driver_core.cpp
  src/dm_mit.cpp
  src/gripper_bridge.cpp)
target_include_directories(skye_driver_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${GENTO_SDK_INCLUDE_DIRS})
target_link_libraries(skye_driver_core PUBLIC ${GENTO_SDK_LIBRARY})
target_compile_definitions(skye_driver_core PUBLIC CMPL_LIN)
set_target_properties(skye_driver_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  BUILD_RPATH "${GENTO_SDK_LIBDIR}"
  INSTALL_RPATH "${GENTO_SDK_LIBDIR}")

add_executable(skye_robot_driver_node src/main.cpp src/driver_node.cpp)
target_include_directories(skye_robot_driver_node PRIVATE include)
target_link_libraries(skye_robot_driver_node skye_driver_core)
ament_target_dependencies(skye_robot_driver_node
  rclcpp sensor_msgs std_msgs std_srvs)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(skye_robot_driver_node "${cpp_typesupport_target}")
set_target_properties(skye_robot_driver_node PROPERTIES
  OUTPUT_NAME skye_robot_driver
  BUILD_RPATH "${GENTO_SDK_LIBDIR}"
  INSTALL_RPATH "${GENTO_SDK_LIBDIR}")

install(TARGETS skye_robot_driver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_driver_core test/test_driver_core.cpp)
  target_link_libraries(test_driver_core skye_driver_core)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
