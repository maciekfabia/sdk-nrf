#
# Copyright (c) 2020 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

cmake_minimum_required(VERSION 3.8)

add_definitions(-Wno-packed-bitfield-compat)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)

zephyr_compile_definitions(CONFIG_ZBOSS_OSIF_LOG_LEVEL=LOG_LEVEL_DBG)

zephyr_include_directories(${ZEPHYR_BASE}/../nrf/subsys/zigbee/osif)
zephyr_include_directories(${ZEPHYR_BASE}/../nrfxlib/zboss/include)
zephyr_include_directories(${ZEPHYR_BASE}/../nrfxlib/zboss/include/osif)
project(NONE)

target_sources(app PRIVATE
  src/main.c
  ${ZEPHYR_BASE}/../nrf/subsys/zigbee/osif/zb_nrf_crypto.c
  ${ZEPHYR_BASE}/../nrf/subsys/zigbee/osif/nrf_ecb_driver.c
)
