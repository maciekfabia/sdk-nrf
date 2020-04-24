/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef ZB_NRF_PLATFORM_H__
#define ZB_NRF_PLATFORM_H__

#include <zboss_api.h>


typedef enum {
	ZIGBEE_EVENT_TX_FAILED,
	ZIGBEE_EVENT_TX_DONE,
	ZIGBEE_EVENT_RX_DONE,
	ZIGBEE_EVENT_APP,
} zigbee_event_t;


/* Function for starting Zigbee thread. */
void zigbee_enable(void);

/**@brief Notify ZBOSS thread about a new event.
 *
 * @param[in] event  Event to notify.
 */
void zigbee_event_notify(zigbee_event_t event);

/**@brief Function which waits for event in case of empty Zigbee stack scheduler queue.
 *
 * @param[in] timeout_ms  Maximum amount of time, in milliseconds for which the ZBOSS task processing may be blocked.
 *
 * @returns The amount of milliseconds that the ZBOSS task was blocked.
 */
u32_t zigbee_event_poll(u32_t timeout_ms);

/**@brief Schedule single-param callback execution and unlock Zigbee task afterwards.

   Schedule execution of function `func' in the main scheduler loop.

   @param func - function to execute
   @param param - callback parameter - usually, but not always ref to packet buffer

   @return RET_OK or RET_OVERFLOW.
 */
static inline zb_ret_t zigbee_schedule_callback(zb_callback_t func, zb_uint8_t param) {
	zb_ret_t ret_code = ZB_SCHEDULE_APP_CALLBACK(func, param);
	if (ret_code == RET_OK) {
		zigbee_event_notify(ZIGBEE_EVENT_APP);
	}
	return ret_code;
}

/**@brief Schedule two-param callback execution and unlock Zigbee task afterwards.

   Schedule execution of function `func' in the main scheduler loop.

   @param func - function to execute
   @param param - zb_uint8_t callback parameter - usually, but not always ref to packet buffer
   @param user_param - zb_uint16_t user parameter - usually, but not always short address

   @return RET_OK or RET_OVERFLOW.
 */
static inline zb_ret_t zigbee_schedule_callback2(zb_callback_t func, zb_uint8_t param, zb_uint16_t user_param) {
	zb_ret_t ret_code = ZB_SCHEDULE_APP_CALLBACK2(func, param, user_param);
	if (ret_code == RET_OK) {
		zigbee_event_notify(ZIGBEE_EVENT_APP);
	}
	return ret_code;
}


/**@brief Schedule alarm - callback to be executed after timeout and unlock Zigbee task afterwards.

   Function will be called via scheduler after timeout expired (maybe, plus some additional time).
   Timer resolution depends on implementation.
   Same callback can be scheduled for execution more then once.

   @param func - function to call via scheduler
   @param param - parameter to pass to the function
   @param timeout_bi - timeout, in beacon intervals

   @return RET_OK or RET_OVERFLOW
 */
static inline zb_ret_t zigbee_schedule_alarm(zb_callback_t func, zb_uint8_t param, zb_time_t run_after) {
	zb_ret_t ret_code = ZB_SCHEDULE_APP_ALARM(func, param, run_after);
	if (ret_code == RET_OK) {
		zigbee_event_notify(ZIGBEE_EVENT_APP);
	}
	return ret_code;
}

#endif /* ZB_NRF_PLATFORM_H__ */
