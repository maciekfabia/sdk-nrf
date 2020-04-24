/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <stdlib.h>

#include <kernel.h>
#include <logging/log.h>
#include <init.h>

#include "zb_nrf_platform.h"
#include "zboss_api.h"
#include "zb_nrf_crypto.h"


LOG_MODULE_REGISTER(zboss_osif, CONFIG_ZBOSS_OSIF_LOG_LEVEL);

/* Signal object to indicate that frame has been received */
static struct k_poll_signal zigbee_sig = K_POLL_SIGNAL_INITIALIZER(zigbee_sig);

/** Global mutex to protect access to the ZBOSS global state.
 *
 * @note Functions for locking/unlocking the mutex are called directly from ZBOSS core,
 *       when the main ZBOSS global variable is accessed.
 */
static K_MUTEX_DEFINE(zigbee_mutex);

K_THREAD_STACK_DEFINE(zboss_stack_area, CONFIG_ZBOSS_DEFAULT_THREAD_STACK_SIZE);
static struct k_thread zboss_thread_data;
static k_tid_t zboss_tid;

static int zigbee_init(struct device *unused)
{
	zb_ieee_addr_t ieee_addr;
	zb_uint32_t channel_mask;

	/* Set Zigbee stack logging level and traffic dump subsystem. */
	ZB_SET_TRACE_LEVEL(CONFIG_ZBOSS_TRACE_LOG_LEVEL);
	ZB_SET_TRACE_MASK(CONFIG_ZBOSS_TRACE_MASK);
	ZB_SET_TRAF_DUMP_OFF();

	/* Initialize Zigbee stack. */
	ZB_INIT("zigbee_thread");

	/* Set device address to the value read from FICR registers. */
	zb_osif_get_ieee_eui64(ieee_addr);
	zb_set_long_address(ieee_addr);

	/* Keep or erase NVRAM to save the network parameters after device reboot or power-off. */
	zb_set_nvram_erase_at_start(ZB_FALSE);

	/* Set channels on which the coordinator will try
	 * to create a new network
	 */
#if defined(CONFIG_ZIGBEE_CHANNEL_SELECTION_MODE_SINGLE)
	channel_mask = (1UL << CONFIG_ZIGBEE_CHANNEL);
#elif defined(CONFIG_ZIGBEE_CHANNEL_SELECTION_MODE_MULTI)
	channel_mask = CONFIG_ZIGBEE_CHANNEL_MASK;
#else
#error Channel mask undefined!
#endif

#if defined(CONFIG_ZIGBEE_ROLE_COORDINATOR)
	zb_set_network_coordinator_role(channel_mask);
#elif defined(CONFIG_ZIGBEE_ROLE_ROUTER)
	zb_set_network_router_role(channel_mask);
#elif defined(CONFIG_ZIGBEE_ROLE_END_DEVICE)
	zb_set_network_ed_role(channel_mask);
#else
#error Zigbee device role undefined!
#endif

	return 0;
}

static void zboss_thread(void *arg1, void *arg2, void *arg3)
{
	zb_ret_t zb_err_code;

	zb_err_code = zboss_start_no_autostart();
	__ASSERT(zb_err_code == RET_OK , "Error when starting ZBOSS stack!");

	while (1) {
		zboss_main_loop_iteration();
	}
}


/**@brief SoC general initialization. */
void zb_osif_init()
{
	static bool platform_inited = false;

	if (platform_inited) {
		return;
	} else {
		platform_inited = true;
	}

#ifdef CONFIG_ZB_HAVE_SERIAL
	/* Initialise serial trace */
	zb_osif_serial_init();
#endif

	/* Initialise random generator */
	zb_osif_rng_init();

	/* Initialise AES ECB */
	zb_osif_aes_init();

#ifdef ZB_USE_SLEEP
	/* Initialise power consumption routines */
	zb_osif_sleep_init();
#endif /*ZB_USE_SLEEP*/
}

void zb_osif_abort(void)
{
	LOG_ERR("Fatal error occurred");
	k_fatal_halt(K_ERR_KERNEL_PANIC);
}

zb_void_t zb_reset(zb_uint8_t param)
{
	ZVUNUSED(param);

	LOG_ERR("Fatal error occurred");
	k_fatal_halt(K_ERR_KERNEL_PANIC);
}


void zb_osif_enable_all_inter(void)
{
	__ASSERT(zb_osif_is_inside_isr() == 0, "Unable to unlock mutex from interrupt context");
	k_mutex_unlock(&zigbee_mutex);
}

void zb_osif_disable_all_inter(void)
{
	__ASSERT(zb_osif_is_inside_isr() == 0, "Unable to lock mutex from interrupt context");
	k_mutex_lock(&zigbee_mutex, K_FOREVER);
}

void zb_osif_busy_loop_delay(zb_uint32_t count)
{
	k_busy_wait(count);
}

zb_bool_t zb_osif_is_inside_isr(void)
{
	return (zb_bool_t)(__get_IPSR() != 0);
}

__weak zb_uint32_t zb_get_utc_time(void)
{
	LOG_ERR("Unable to obtain UTC time. Please implement zb_get_utc_time in your application to provide the current UTC time.");
	return ZB_TIME_BEACON_INTERVAL_TO_MSEC(ZB_TIMER_GET()) / 1000;
}

/**@brief Read IEEE long address from FICR registers. */
void zb_osif_get_ieee_eui64(zb_ieee_addr_t ieee_eui64)
{
	uint64_t factoryAddress;

	/* Read random address from FICR. */
	factoryAddress = (uint64_t)NRF_FICR->DEVICEID[0] << 32;
	factoryAddress |= NRF_FICR->DEVICEID[1];

	/* Set constant manufacturer ID to use MAC compression mechanisms. */
	factoryAddress &= 0x000000FFFFFFFFFFLL;
	factoryAddress |= (uint64_t)(CONFIG_ZIGBEE_VENDOR_OUI) << 40;

	memcpy(ieee_eui64, &factoryAddress, sizeof(factoryAddress));
}



void zigbee_event_notify(zigbee_event_t event)
{
	k_poll_signal_raise(&zigbee_sig, event);
}

u32_t zigbee_event_poll(u32_t timeout_ms)
{
	/* Configure event/signals to wait for in wait_for_event function */
    static struct k_poll_event wait_events[] = {
		K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
								 K_POLL_MODE_NOTIFY_ONLY,
								 &zigbee_sig),
	};

	unsigned int signaled;
	int result;
	s64_t time_stamp = k_uptime_get();

	k_poll(wait_events, 1, timeout_ms);

	k_poll_signal_check(&zigbee_sig, &signaled, &result);
	if (signaled) {
		k_poll_signal_reset(&zigbee_sig);

		LOG_DBG("Received new Zigbee event: 0x%02x", result);
	}

	return k_uptime_delta(&time_stamp);
}

void zigbee_enable(void)
{
	zboss_tid = k_thread_create(&zboss_thread_data, zboss_stack_area,
				K_THREAD_STACK_SIZEOF(zboss_stack_area),
				zboss_thread,
				NULL, NULL, NULL,
				CONFIG_ZBOSS_DEFAULT_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&zboss_thread_data, "zboss");
}

SYS_INIT(zigbee_init, POST_KERNEL, CONFIG_ZBOSS_DEFAULT_THREAD_PRIORITY);
