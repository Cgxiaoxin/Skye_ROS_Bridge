cmake_minimum_required(VERSION 3.16)
project(gento_robot_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(GENTO_SDK_ROOT "/data/coding/tianji/tianji-robot-SDK-Gento_Skye-Luna/C_SDK")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_library(GENTO_SDK_LIBRARY NAMES GentoSDK PATHS ${GENTO_SDK_ROOT} NO_DEFAULT_PATH REQUIRED)

set(GENTO_SDK_INCLUDE_DIRS
  ${GENTO_SDK_ROOT}/L1Robot
  ${GENTO_SDK_ROOT}/Common
  ${GENTO_SDK_ROOT}/L0Control
  ${GENTO_SDK_ROOT}/FileClient
  ${GENTO_SDK_ROOT}/Kinematics
  ${GENTO_SDK_ROOT}/Kinematics/ArmKinematics
  ${GENTO_SDK_ROOT}/Kinematics/BaseMath
  ${GENTO_SDK_ROOT}/Kinematics/DynaIdent
  ${GENTO_SDK_ROOT}/Kinematics/KineCommon
  ${GENTO_SDK_ROOT}/Kinematics/MotionPlanner
  ${GENTO_SDK_ROOT}/Kinematics/SkyeBodyKinematics)

add_library(gento_driver_core src/driver_core.cpp)
target_include_directories(gento_driver_core PUBLIC include ${GENTO_SDK_INCLUDE_DIRS})
target_link_libraries(gento_driver_core PUBLIC ${GENTO_SDK_LIBRARY})
target_compile_definitions(gento_driver_core PUBLIC CMPL_LIN)
set_target_properties(gento_driver_core PROPERTIES BUILD_RPATH ${GENTO_SDK_ROOT})

add_library(gento_robot_driver_node_library src/gento_robot_driver_node.cpp)
target_include_directories(gento_robot_driver_node_library PUBLIC include)
target_link_libraries(gento_robot_driver_node_library gento_driver_core)
ament_target_dependencies(gento_robot_driver_node_library rclcpp sensor_msgs std_srvs)
set_target_properties(gento_robot_driver_node_library PROPERTIES
  BUILD_RPATH ${GENTO_SDK_ROOT}
  INSTALL_RPATH ${GENTO_SDK_ROOT})

add_executable(gento_robot_driver src/main.cpp)
target_link_libraries(gento_robot_driver gento_robot_driver_node_library)
ament_target_dependencies(gento_robot_driver rclcpp)
set_target_properties(gento_robot_driver PROPERTIES
  BUILD_RPATH ${GENTO_SDK_ROOT}
  INSTALL_RPATH ${GENTO_SDK_ROOT})

ament_add_gtest(test_sdk_abi test/test_sdk_abi.cpp)
target_include_directories(test_sdk_abi PRIVATE ${GENTO_SDK_INCLUDE_DIRS})
target_link_libraries(test_sdk_abi ${GENTO_SDK_LIBRARY})
target_compile_definitions(test_sdk_abi PRIVATE CMPL_LIN)
set_target_properties(test_sdk_abi PROPERTIES BUILD_RPATH ${GENTO_SDK_ROOT})

ament_add_gtest(test_driver_core test/test_driver_core.cpp)
target_link_libraries(test_driver_core gento_driver_core)
set_target_properties(test_driver_core PROPERTIES BUILD_RPATH ${GENTO_SDK_ROOT})

ament_add_gtest(test_node_interfaces test/test_node_interfaces.cpp)
target_link_libraries(test_node_interfaces gento_robot_driver_node_library)
ament_target_dependencies(test_node_interfaces rclcpp sensor_msgs std_srvs)
set_target_properties(test_node_interfaces PROPERTIES BUILD_RPATH ${GENTO_SDK_ROOT})
set_tests_properties(test_node_interfaces PROPERTIES ENVIRONMENT "ROS_LOG_DIR=/tmp")

install(TARGETS
  gento_robot_driver
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
