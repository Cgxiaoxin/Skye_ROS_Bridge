cmake_minimum_required(VERSION 3.16)
project(gento_robot_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Repo: Skye_ROS_Bridge/third_party/gento_sdk
set(SKYE_BRIDGE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../..")
set(GENTO_SDK_ROOT "${SKYE_BRIDGE_ROOT}/third_party/gento_sdk")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(GENTO_SDK_LIBDIR "${GENTO_SDK_ROOT}/lib/aarch64")
else()
  set(GENTO_SDK_LIBDIR "${GENTO_SDK_ROOT}/lib/x86_64")
endif()

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

add_library(gento_driver_core src/driver_core.cpp)
target_include_directories(gento_driver_core PUBLIC include ${GENTO_SDK_INCLUDE_DIRS})
target_link_libraries(gento_driver_core PUBLIC ${GENTO_SDK_LIBRARY})
target_compile_definitions(gento_driver_core PUBLIC CMPL_LIN)
set_target_properties(gento_driver_core PROPERTIES BUILD_RPATH ${GENTO_SDK_LIBDIR})

add_library(gento_robot_driver_node_library src/gento_robot_driver_node.cpp)
target_include_directories(gento_robot_driver_node_library PUBLIC include)
target_link_libraries(gento_robot_driver_node_library gento_driver_core)
ament_target_dependencies(gento_robot_driver_node_library rclcpp sensor_msgs std_srvs)
set_target_properties(gento_robot_driver_node_library PROPERTIES
  BUILD_RPATH ${GENTO_SDK_LIBDIR}
  INSTALL_RPATH ${GENTO_SDK_LIBDIR})

add_executable(gento_robot_driver src/main.cpp)
target_link_libraries(gento_robot_driver gento_robot_driver_node_library)
ament_target_dependencies(gento_robot_driver rclcpp)
set_target_properties(gento_robot_driver PROPERTIES
  BUILD_RPATH ${GENTO_SDK_LIBDIR}
  INSTALL_RPATH ${GENTO_SDK_LIBDIR})

ament_add_gtest(test_sdk_abi test/test_sdk_abi.cpp)
target_include_directories(test_sdk_abi PRIVATE ${GENTO_SDK_INCLUDE_DIRS})
target_link_libraries(test_sdk_abi ${GENTO_SDK_LIBRARY})
target_compile_definitions(test_sdk_abi PRIVATE CMPL_LIN)
set_target_properties(test_sdk_abi PROPERTIES BUILD_RPATH ${GENTO_SDK_LIBDIR})

ament_add_gtest(test_driver_core test/test_driver_core.cpp)
target_link_libraries(test_driver_core gento_driver_core)
set_target_properties(test_driver_core PROPERTIES BUILD_RPATH ${GENTO_SDK_LIBDIR})

ament_add_gtest(test_node_interfaces test/test_node_interfaces.cpp)
target_link_libraries(test_node_interfaces gento_robot_driver_node_library)
ament_target_dependencies(test_node_interfaces rclcpp sensor_msgs std_srvs)
set_target_properties(test_node_interfaces PROPERTIES BUILD_RPATH ${GENTO_SDK_LIBDIR})
set_tests_properties(test_node_interfaces PROPERTIES ENVIRONMENT "ROS_LOG_DIR=/tmp")

install(TARGETS
  gento_robot_driver
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()