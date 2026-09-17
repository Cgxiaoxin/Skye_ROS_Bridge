# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(skye_factr_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md THIRD_PARTY_NOTICES.md LICENSE
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
