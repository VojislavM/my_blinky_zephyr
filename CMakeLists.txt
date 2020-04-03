# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8.2)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(my_blinky)

target_sources(app PRIVATE src/main.c)

# Include source directory
zephyr_include_directories(src)

# Application directories
add_subdirectory(src/sensors)
