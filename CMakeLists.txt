# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

# This sample is specific to x_nucleo_iks01a2 shield. Enforce -DSHIELD option
set(SHIELD x_nucleo_iks01a2)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(x_nucleo_iks01a2)

zephyr_compile_definitions(
  -DMETAL_MAX_DEVICE_REGIONS=2
  -DVIRTIO_SLAVE_ONLY
  )
target_include_directories(app PRIVATE $ENV{ZEPHYR_BASE}/drivers)
target_include_directories(app PRIVATE ${LIBMETAL_INCLUDE_DIR} ${OPENAMP_INCLUDE_DIR} ${PLATFORM_DIR})

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
