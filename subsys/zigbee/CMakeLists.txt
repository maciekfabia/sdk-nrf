#
# Copyright (c) 2020 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

zephyr_include_directories(common)
zephyr_include_directories(osif)
zephyr_include_directories(zb_error)

# Source files
zephyr_sources(osif/zb_nrf_platform.c)
zephyr_sources(osif/zb_nrf_logger.c)
zephyr_sources(osif/zb_nrf_nvram.c)
zephyr_sources(osif/zb_nrf_timer.c)
zephyr_sources(osif/zb_nrf_transceiver.c)
zephyr_sources(osif/zb_nrf_crypto.c)
zephyr_sources(osif/zb_nrf_pwr_mgmt.c)
zephyr_sources(osif/nrf_ecb_driver.c)
zephyr_sources(common/zigbee_helpers.c)
zephyr_sources(common/zigbee_logger_eprxzcl.c)
