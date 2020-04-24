#
# Copyright (c) 2020 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

cmake_minimum_required(VERSION 3.8)

add_definitions(-Wno-packed-bitfield-compat)
add_definitions(-DTIMER_DEFAULT_CONFIG_IRQ_PRIORITY=6)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)

project(NONE)

target_sources(app PRIVATE
  src/main.c
)
