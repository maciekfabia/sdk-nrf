#
# Copyright (c) 2020 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

cmake_minimum_required(VERSION 3.8)

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/dts.overlay")
  set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/dts.overlay")
endif()

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD}.overlay")
  set(DTC_OVERLAY_FILE "${DTC_OVERLAY_FILE} ${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD}.overlay")
endif()

include($ENV{ZEPHYR_BASE}/../nrf/cmake/boilerplate.cmake)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)

project("Zigbee light bulb")

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
)
# NORDIC SDK APP END
