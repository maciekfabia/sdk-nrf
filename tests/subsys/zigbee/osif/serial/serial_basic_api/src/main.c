/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */
#include <ztest.h>
#include <zboss_api.h>

#include <zb_nrf_serial.c>


#define TEST_UART_BUF_LEN 256


/* The following line declares uart_tx_buf_t type. */
ZB_RING_BUFFER_DECLARE(uart_tx_test_buf, zb_uint8_t, TEST_UART_BUF_LEN);
static uart_tx_test_buf_t uart_tx_test_buf;
static u8_t uart_rx_test_buf[TEST_UART_BUF_LEN];
static u32_t uart_rx_buf_cnt;


static size_t fill_with_test_data(u8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		buf[i] = (i % 0x100);
	}

	return len;
}

static void uart_char_handler(zb_uint8_t ch)
{
	if (uart_rx_buf_cnt < sizeof(uart_rx_test_buf)) {
		uart_rx_test_buf[uart_rx_buf_cnt++] = ch;
	}
}


/**
 * Verify that the driver is found and initialized by the OSIF implementation.
 */
static void test_init(void)
{
	zb_osif_serial_init();
	zassert_not_null(uart_dev, "UART device was not initialized");
}

/**
 * Verify that the OSIF implementation is ready for transmission and reception.
 */
static void test_tx_rx_setup(void)
{
	zb_osif_set_uart_byte_received_cb(uart_char_handler);
	zb_osif_set_user_io_buffer((zb_byte_array_t *)&uart_tx_test_buf,
				   TEST_UART_BUF_LEN);

	zassert_equal_ptr(char_handler, uart_char_handler,
			  "The UART RX callback in not properly set");
}

/**
 * Verify simple transmission and reception.
 */
static void test_tx_rx(void)
{
	zb_uint8_t test_serial_data[TEST_UART_BUF_LEN] = { 0 };
	size_t bytes_filled = fill_with_test_data(test_serial_data,
						  sizeof(test_serial_data));

	/* Reset rx buffer. */
	uart_rx_buf_cnt = 0;
	memset(uart_rx_test_buf, 0, sizeof(uart_rx_test_buf));

	zb_osif_serial_put_bytes(test_serial_data, bytes_filled);
	k_sleep(K_MSEC(CONFIG_ZIGBEE_UART_PARTIAL_RX_TIMEOUT + 100));

	zassert_equal(bytes_filled, uart_rx_buf_cnt,
		      "The number of transmitted and received bytes differ");
	zassert_equal(
		memcmp(test_serial_data, uart_rx_test_buf, uart_rx_buf_cnt),
		0,
		"Transmitted and received bytes differ");
}

/**
 * Verify UART sleep API.
 */
static void test_sleep(void)
{
	/*
	 * Since power optimization is not yet implemented, simply verify
	 * that the device can withstand sleep state and continue reception.
	 */
	zb_osif_uart_sleep();
	zb_osif_uart_wake_up();
}

/**
 * Verify reception of odd-size transmissions.
 */
static void test_undiv_tx_rx(void)
{
	zb_uint8_t test_serial_data[TEST_UART_BUF_LEN] = { 0 };
	size_t bytes_filled = fill_with_test_data(test_serial_data,
						  sizeof(test_serial_data));

	/* Reset rx buffer. */
	uart_rx_buf_cnt = 0;
	memset(uart_rx_test_buf, 0, sizeof(uart_rx_test_buf));

	for (int i = 0; i < 3; i++) {
		size_t bytes_sent = (bytes_filled / 3) * (i + 1);

		zb_osif_serial_put_bytes(
			&test_serial_data[bytes_filled * i / 3],
			bytes_filled / 3);
		k_sleep(K_MSEC(CONFIG_ZIGBEE_UART_PARTIAL_RX_TIMEOUT + 100));
		zassert_equal(bytes_sent, uart_rx_buf_cnt,
			"The number of transmitted and received bytes differ");
	}
	zassert_equal(
		memcmp(test_serial_data, uart_rx_test_buf, uart_rx_buf_cnt),
		0,
		"Transmitted and received bytes differ");
}

/**
 * Verify reception of several, continuous transmissions.
 */
static void test_multi_tx_rx(void)
{
	zb_uint8_t test_serial_data[TEST_UART_BUF_LEN] = { 0 };
	size_t bytes_filled = fill_with_test_data(test_serial_data,
						  sizeof(test_serial_data));

	/* Reset rx buffer. */
	uart_rx_buf_cnt = 0;
	memset(uart_rx_test_buf, 0, sizeof(uart_rx_test_buf));

	for (int i = 0; i < 4; i++) {
		size_t bytes_sent = bytes_filled * i / 4;

		zb_osif_serial_put_bytes(&test_serial_data[bytes_sent],
					 bytes_filled / 4);
	}

	k_sleep(K_MSEC(CONFIG_ZIGBEE_UART_PARTIAL_RX_TIMEOUT + 100));
	zassert_equal(bytes_filled, uart_rx_buf_cnt,
		"The number of transmitted and received bytes is not equal");
	zassert_equal(
		memcmp(test_serial_data, uart_rx_test_buf, uart_rx_buf_cnt),
		0,
		"Transmitted and received bytes differ");
}

void test_main(void)
{
	printk("---------------------------------------------------\r\n");
	printk("PRECONDITION: Short UART1 pins before running tests\r\n");
	printk("---------------------------------------------------\r\n");

	/* Enable zigbee callbacks implemented in zigbee_scheduler mock. */
	zigbee_enable();

	ztest_test_suite(nrf_osif_serial_tests,
			 ztest_unit_test(test_init),
			 ztest_unit_test(test_tx_rx_setup),
			 ztest_unit_test(test_tx_rx),
			 ztest_unit_test(test_sleep),
			 ztest_unit_test(test_undiv_tx_rx),
			 ztest_unit_test(test_multi_tx_rx)
			 );

	ztest_run_test_suite(nrf_osif_serial_tests);
}
