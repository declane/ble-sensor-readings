#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(peripheral_uart)

# NORDIC SDK APP START
target_include_directories(app PRIVATE "src/app_ble")
target_include_directories(app PRIVATE "src/sensors")

target_sources(app PRIVATE
  src/main.c
  src/app_ble/ble_manager.c
  src/app_ble/ble_manager.h
  src/sensors/gy_521.c
  src/sensors/gy_521.h
)

# NORDIC SDK APP END
