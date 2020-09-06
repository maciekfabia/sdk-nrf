/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <kernel.h>
#include <logging/log.h>
#include <nrf_802154.h>
#include <nrf_802154_const.h>
#include <zboss_api.h>
#include <zb_macll.h>
#include <zb_transceiver.h>
#include "zb_nrf_platform.h"

/* ZEPHYR API */
#include <device.h>
#include <net/ieee802154_radio.h>
/* ZEPHYR API - END */

/*L2 layer*/
#include <net/net_core.h>
#include <net/net_l2.h>
#include <net/net_if.h>
#include <net/net_pkt.h>
/*L2 layer end*/

#if !defined NRF_802154_FRAME_TIMESTAMP_ENABLED ||                             \
	!NRF_802154_FRAME_TIMESTAMP_ENABLED
#warning Must define NRF_802154_FRAME_TIMESTAMP_ENABLED!
#endif

#define NRF5_FCS_LENGTH   2

LOG_MODULE_DECLARE(zboss_osif, CONFIG_ZBOSS_OSIF_LOG_LEVEL);

BUILD_ASSERT(IS_ENABLED(CONFIG_NET_PKT_TIMESTAMP), "Timestamp is required");

/* Definition of FIFO queue entry for the received frame */
/* OLD */
#if 0
typedef struct nrf_802154_rx_frame {
	void	*fifo_reserved; /* 1st word reserved for the kernel's use. */
	uint8_t	*data; /* Pointer to a received frame. */
	int8_t	power; /* Last received frame RSSI value. */
	uint8_t	lqi; /* Last received frame LQI value. */
	uint32_t	time; /* Timestamp in microseconds. */
	bool	pending_bit;
} rx_frame_t;
#endif

enum ieee802154_radio_state {
	RADIO_802154_STATE_SLEEP,
	RADIO_802154_STATE_ACTIVE,
	RADIO_802154_STATE_RECEIVE,
};

struct ieee802154_state_cache {
	int8_t power;
	enum ieee802154_radio_state radio_state;
	bool pan_coordinator;
};

static struct ieee802154_state_cache state_cache = {
	.power = SCHAR_MIN,
	.radio_state = RADIO_802154_STATE_SLEEP,
	.pan_coordinator = false,
};

/* RX fifo queue. */
static struct k_fifo rx_fifo;

/* Array of pointers to received frames and their metadata.
 *
 * Keep one additional element in case a new frame is received after
 * nrf_802154_buffer_free_raw(..) is called and before the rx_frame->data
 * is set back to NULL.
 */
/* OLD */
//static rx_frame_t rx_frames[NRF_802154_RX_BUFFERS + 1];

struct rx_fifo_item {
	void *fifo_reserved;
	struct net_pkt *pkt;
};

static struct rx_fifo_item rx_fifo_buf[NRF_802154_RX_BUFFERS + 1];

#define ACK_PKT_LENGTH 3
#define PHR_LENGTH 1

static volatile uint8_t ack_frame_buf[ACK_PKT_LENGTH + PHR_LENGTH];
static volatile uint8_t *ack_frame;

static struct {
	/* Semaphore for waiting for end of energy detection procedure. */
	struct k_sem sem;
	volatile bool failed;   /* Energy detection procedure failed. */
	volatile uint32_t time_ms; /* Duration of energy detection procedure. */
	volatile uint8_t rssi_val; /* Detected energy level. */
} energy_detect;

static volatile bool acked_with_pending_bit;

/* ZEPHYR API */
static struct device *radio_dev;
static struct ieee802154_radio_api *radio_api;
static struct net_if *radio_if;
/* ZEPHYR API - END */

#if 0 /* OLD */
#if !CONFIG_MPSL
static void nrf5_radio_irq(void *arg)
{
	ARG_UNUSED(arg);

	nrf_802154_radio_irq_handler();
}
#endif

static void nrf5_irq_config(void)
{
#if !CONFIG_MPSL
	IRQ_CONNECT(RADIO_IRQn, NRF_802154_IRQ_PRIORITY, nrf5_radio_irq, NULL,
		    0);
	irq_enable(RADIO_IRQn);
#endif
}
#endif /* OLD - END */

/* Initializes the transceiver. */
void zb_trans_hw_init(void)
{
	struct ieee802154_config config;

	/* ZEPHYR API */
	radio_dev = device_get_binding(CONFIG_NET_CONFIG_IEEE802154_DEV_NAME);
	__ASSERT_NO_MSG(radio_dev != NULL);

	radio_api = (struct ieee802154_radio_api *)radio_dev->api;
	__ASSERT_NO_MSG(radio_api != NULL);
	/* ZEPHYR API - END */

	/* OLD */
	/* done by driver init macro*/
	//nrf_802154_init();

	/* OLD -> NEW */
	//nrf_802154_auto_ack_set(ZB_TRUE);
	//nrf_802154_src_addr_matching_method_set(
	//	NRF_802154_SRC_ADDR_MATCH_ZIGBEE);
	config.auto_ack_fpb.enabled = true;
	config.auto_ack_fpb.mode = IEEE802154_FPB_ADDR_MATCH_ZIGBEE;
	radio_api->configure(radio_dev,
			     IEEE802154_CONFIG_AUTO_ACK_FPB,
			     &config);


	/* WORKAROUND: Send continuous carrier to leave the sleep state
	 *             and request the HFCLK.
	 *             Manually wait for HFCLK to synchronize by sleeping
	 *             for 1 ms afterwards.
	 *
	 * The bug causes the Radio Driver to start transmitting
	 * frames while the HFCLK is still starting.
	 * As a result, the Radio Driver, as well as the ZBOSS,
	 * is notified that the frame was sent, but it was not, because
	 * the analog part of the radio was not working at that time.
	 * The carrier will not be transmitted for the same reason.
	 * It is placed here just to trigger the proper set of events
	 * in Radio Driver.
	 *
	 * Since the coordinator/router/non-sleepy end device does
	 * not reenter sleep state, it is enough to apply this
	 * procedure here. For SED it has to be repeated every time
	 * the Radio leaves sleep state.
	 *
	 * More info: https://github.com/zephyrproject-rtos/zephyr/issues/20712
	 * JIRA issue: KRKNWK-5533
	 */
	/*(void)nrf_802154_sleep(); */

	/* OLD; maybe no need for workaround in shim? */
	//(void)nrf_802154_continuous_carrier();
	//k_sleep(K_MSEC(1));

	//net_if_flag_set(radio_if, NET_IF_UP);

#if 0
	/* OLD -> NEW */
	//(void)nrf_802154_receive();
	radio_api->start(radio_dev);
	state_cache.radio_state = RADIO_802154_STATE_RECEIVE;

	k_fifo_init(&rx_fifo);
	k_sem_init(&energy_detect.sem, 1, 1);

	/* OLD; shim configures irq thru driver macro? */
	//nrf5_irq_config();
#endif

	LOG_DBG("The 802.15.4 radio initialized.");
}

/* Sets the PAN ID used by the device. */
void zb_trans_set_pan_id(zb_uint16_t pan_id)
{
	struct ieee802154_filter filter = { .pan_id = pan_id };

	LOG_DBG("Function: %s, PAN ID: 0x%x", __func__, pan_id);

	/* OLD -> NEW*/
	//nrf_802154_pan_id_set((zb_uint8_t *)(&pan_id));
	radio_api->filter(radio_dev, true, IEEE802154_FILTER_TYPE_PAN_ID, &filter);
}

/* Sets the long address in the radio driver. */
void zb_trans_set_long_addr(zb_ieee_addr_t long_addr)
{
	struct ieee802154_filter filter = { .ieee_addr = long_addr };

	LOG_DBG("Function: %s, long addr: 0x%llx", __func__, (uint64_t)*long_addr);

	/* OLD -> NEW*/
	//nrf_802154_extended_address_set(long_addr);
	radio_api->filter(radio_dev,
			  true,
			  IEEE802154_FILTER_TYPE_IEEE_ADDR,
			  &filter);
}

/* Sets the short address of the device. */
void zb_trans_set_short_addr(zb_uint16_t addr)
{
	struct ieee802154_filter filter = { .short_addr = addr };

	LOG_DBG("Function: %s, 0x%x", __func__, addr);

	/* OLD -> NEW*/
	//nrf_802154_short_address_set((uint8_t *)(&addr));
	radio_api->filter(radio_dev,
			  true,
			  IEEE802154_FILTER_TYPE_SHORT_ADDR,
			  &filter);
}

/* energy detection callback */
static void energy_scan_done(struct device *dev, int16_t max_ed)
{
	ARG_UNUSED(dev);

	if (max_ed == SHRT_MAX) {
		energy_detect.failed = true;
	} else {
		energy_detect.rssi_val = (max_ed - ED_MIN_DBM) * ED_RESULT_FACTOR;
	}
	k_sem_give(&energy_detect.sem);
}

/* Start the energy detection procedure */
void zb_trans_start_get_rssi(zb_uint8_t scan_duration_bi)
{
	//uint32_t time_us = ZB_TIME_BEACON_INTERVAL_TO_USEC(scan_duration_bi);
	//uint32_t time_ms = ZB_TIME_BEACON_INTERVAL_TO_MSEC(scan_duration_bi);

	energy_detect.failed = false;

#if 0
	energy_detect.time_ms = time_us / 1000U;  /* OLD -> NEW; us -> ms*/

	/* if there's fractional part of duration round it up */
	if (time_us % 1000U > 0U) {
		energy_detect.time_ms++;
	}
#endif
	energy_detect.time_ms =
		ZB_TIME_BEACON_INTERVAL_TO_MSEC(scan_duration_bi);

	LOG_DBG("Function: %s, scan duration: %d", __func__,
		energy_detect.time_ms);
	k_sem_take(&energy_detect.sem, K_FOREVER);

	/* OLD -> NEW*/
	//while (!nrf_802154_energy_detection(energy_detect.time_ms)) {
	//	k_usleep(500);
	//}
	while (radio_api->ed_scan(radio_dev,
				  energy_detect.time_ms,
				  energy_scan_done)) {
		k_usleep(500);
	}
}

/* Waiting for the end of energy detection procedure and reads the RSSI */
void zb_trans_get_rssi(zb_uint8_t *rssi_value_p)
{
	LOG_DBG("Function: %s", __func__);

	/*Wait until the ED scan finishes.*/
	while (1) {
		k_sem_take(&energy_detect.sem, K_FOREVER);
		if (energy_detect.failed == false) {
			*rssi_value_p = energy_detect.rssi_val;
			LOG_DBG("Energy detected: %d", *rssi_value_p);
			break;
		}
		/* Try again */
		LOG_DBG("Energy detect failed, tries again");
		energy_detect.failed = false;

		/* OLD -> NEW*/
		//while (!nrf_802154_energy_detection(energy_detect.time_ms)) {
		//	k_usleep(500);
		//}
		while (radio_api->ed_scan(radio_dev,
					  energy_detect.time_ms,
					  energy_scan_done)) {
			k_usleep(500);
		}
	}
	k_sem_give(&energy_detect.sem);
}

/* Set channel and go to the normal (not ed scan) mode */
void zb_trans_set_channel(zb_uint8_t channel_number)
{
	LOG_DBG("Function: %s, channel number: %d", __func__, channel_number);

	/* OLD -> NEW*/
	//nrf_802154_channel_set(channel_number);
	radio_api->set_channel(radio_dev, channel_number);
	/*assert return value?*/
}

/* Sets the transmit power. */
void zb_trans_set_tx_power(zb_int8_t power)
{
	LOG_DBG("Function: %s, power: %d", __func__, power);
	/* OLD -> NEW*/
	//nrf_802154_tx_power_set(power);
	radio_api->set_txpower(radio_dev, power);
	state_cache.power = power;
}

/* Gets the currently set transmit power. */
void zb_trans_get_tx_power(zb_int8_t *power)
{
	/* OLD -> NEW*/
	//*power = (zb_int8_t)nrf_802154_tx_power_get();
	*power = (zb_int8_t)state_cache.power;

	LOG_DBG("Function: %s, power: %d", __func__, *power);
}

/* Configures the device as the PAN coordinator. */
void zb_trans_set_pan_coord(zb_bool_t enabled)
{
	struct ieee802154_config config = { .pan_coordinator = enabled };

	LOG_DBG("Function: %s, enabled: %d", __func__, enabled);

	/* OLD -> NEW*/
	//nrf_802154_pan_coord_set((bool)enabled);
	radio_api->configure(radio_dev,
			     IEEE802154_CONFIG_PAN_COORDINATOR,
			     &config);
	state_cache.pan_coordinator = enabled;
}

/* Enables or disables the automatic acknowledgments (auto ACK) */
void zb_trans_set_auto_ack(zb_bool_t enabled)
{
	struct ieee802154_config config = {
		.auto_ack_fpb = {
			.enabled = enabled,
			.mode = IEEE802154_FPB_ADDR_MATCH_ZIGBEE}
	};

	LOG_DBG("Function: %s, enabled: %d", __func__, enabled);

	/* OLD -> NEW*/
	//nrf_802154_auto_ack_set((bool)enabled);
	radio_api->configure(radio_dev,
			     IEEE802154_CONFIG_AUTO_ACK_FPB,
			     &config);
}

/* Enables or disables the promiscuous radio mode. */
void zb_trans_set_promiscuous_mode(zb_bool_t enabled)
{
	struct ieee802154_config config = {
		.promiscuous = enabled
	};

	LOG_DBG("Function: %s, enabled: %d", __func__, enabled);

	/* OLD -> NEW*/
	//nrf_802154_promiscuous_set((bool)enabled);
	radio_api->configure(radio_dev,
			     IEEE802154_CONFIG_PROMISCUOUS,
			     &config);
}

/* Changes the radio state to receive. */
void zb_trans_enter_receive(void)
{
	LOG_DBG("Function: %s", __func__);

	/* OLD -> NEW*/
	//(void)nrf_802154_receive();
	radio_api->start(radio_dev);
	state_cache.radio_state = RADIO_802154_STATE_RECEIVE;
}

/* Changes the radio state to sleep. */
void zb_trans_enter_sleep(void)
{
	LOG_DBG("Function: %s", __func__);

	/* OLD -> NEW*/
	//(void)nrf_802154_sleep_if_idle();
	(void)radio_api->stop(radio_dev);
}

/* Returns ZB_TRUE if radio is in receive state, otherwise ZB_FALSE */
zb_bool_t zb_trans_is_receiving(void)
{
	/* OLD -> NEW*/
	//zb_bool_t is_receiv =
	//	(nrf_802154_state_get() == NRF_802154_STATE_RECEIVE) ?
	//		ZB_TRUE : ZB_FALSE;
	zb_bool_t is_receiv =
		(state_cache.radio_state == RADIO_802154_STATE_RECEIVE) ?
			ZB_TRUE : ZB_FALSE;

	LOG_DBG("Function: %s, is receiv: %d", __func__, is_receiv);
	return is_receiv;
}

/* Returns ZB_TRUE if radio is ON or ZB_FALSE if is in sleep state. */
zb_bool_t zb_trans_is_active(void)
{
	/* OLD -> NEW*/
	//zb_bool_t is_active =
	//	(nrf_802154_state_get() != NRF_802154_STATE_SLEEP) ?
	//		ZB_TRUE : ZB_FALSE;
	zb_bool_t is_active =
		(state_cache.radio_state != RADIO_802154_STATE_SLEEP) ?
			ZB_TRUE : ZB_FALSE;

	LOG_DBG("Function: %s, is active: %d", __func__, is_active);
	return is_active;
}

zb_bool_t zb_trans_transmit(zb_uint8_t wait_type, zb_time_t tx_at,
			    zb_uint8_t *tx_buf, zb_uint8_t current_channel)
{
	LOG_DBG("Function: %s, channel: %d", __func__, current_channel);

	struct net_buf frag = {
		.frags = NULL,
		.b = { .data = &tx_buf[1],
		       .len = tx_buf[0] - NRF5_FCS_LENGTH,
		       .size = tx_buf[0] - NRF5_FCS_LENGTH,
		       .__buf = &tx_buf[1]
		}
	};
	int ret;
	zb_bool_t transmit_status = ZB_FALSE;

#ifndef ZB_ENABLE_ZGP_DIRECT
	ARG_UNUSED(tx_at);
	ARG_UNUSED(current_channel);
#endif

	ack_frame = NULL;

	switch (wait_type) {
	case ZB_MAC_TX_WAIT_CSMACA:
/* OLD -> NEW*/
//#if !NRF_802154_CSMA_CA_ENABLED
//		while (!transmit_status) {
//			transmit_status = (zb_bool_t)nrf_802154_transmit_raw(
//				tx_buf, ZB_FALSE);
//		}
//		break;
//#else
//		transmit_status = ZB_TRUE;
//		nrf_802154_transmit_csma_ca_raw(tx_buf);
//#endif
//		break;
		radio_api->tx(radio_dev,
			      IEEE802154_TX_MODE_CSMA_CA,
			      NULL,
			      &frag);
		transmit_status = ZB_TRUE;
		break;

#ifdef ZB_ENABLE_ZGP_DIRECT
	case ZB_MAC_TX_WAIT_ZGP:
		LOG_ERR("ZB_MAC_TX_WAIT_ZGP - not implemented in osif, TBD");
		break;
#endif
	case ZB_MAC_TX_WAIT_NONE:
		/* First transmit attempt without CCA. */

		/* OLD -> NEW*/
		//transmit_status = (zb_bool_t)nrf_802154_transmit_raw(
		//	tx_buf, ZB_FALSE);
		ret = radio_api->tx(radio_dev,
			      IEEE802154_TX_MODE_DIRECT,
			      NULL,
			      &frag);
		transmit_status = (ret == 0) ? ZB_TRUE : ZB_FALSE;
		break;
	default:
		LOG_DBG("Illegal wait_type parameter: %d", wait_type);
		ZB_ASSERT(0);
		break;
	}

	if (transmit_status == ZB_TRUE) {
		/* ack_frame is overwritten if ack frame was received */
		zb_macll_transmitted_raw(ack_frame);

		/* Raise signal to indicate radio event */
		zigbee_event_notify(ZIGBEE_EVENT_TX_DONE);
	} else {
		/* NEW: replace with the actual radio driver error */
		uint8_t error = NRF_802154_TX_ERROR_NO_ACK;

		switch (error) {
		case NRF_802154_TX_ERROR_NO_MEM:
		case NRF_802154_TX_ERROR_ABORTED:
		case NRF_802154_TX_ERROR_TIMESLOT_DENIED:
		case NRF_802154_TX_ERROR_TIMESLOT_ENDED:
		case NRF_802154_TX_ERROR_BUSY_CHANNEL:
			zb_macll_transmit_failed(ZB_TRANS_CHANNEL_BUSY_ERROR);
			break;

		case NRF_802154_TX_ERROR_INVALID_ACK:
		case NRF_802154_TX_ERROR_NO_ACK:
			zb_macll_transmit_failed(ZB_TRANS_NO_ACK);
			break;
		}

		/* Raise signal to indicate radio event */
		zigbee_event_notify(ZIGBEE_EVENT_TX_FAILED);
	}

	return transmit_status;
}

/* Notifies the driver that the buffer containing the received frame
 * is not used anymore
 */
void zb_trans_buffer_free(zb_uint8_t *buf)
{
	/* The buffer containing the released frame is freed
	 * in 802.15.4 shim driver
	 */
	ARG_UNUSED(buf);

	LOG_DBG("Function: %s", __func__);
	//nrf_802154_buffer_free_raw(buf);
}

zb_bool_t zb_trans_set_pending_bit(zb_uint8_t *addr, zb_bool_t value,
				   zb_bool_t extended)
{
	struct ieee802154_config config = {
		.ack_fpb = {
			.addr = addr,
			.extended = extended,
			.enabled = !value
		}
	};
	int ret;

	LOG_DBG("Function: %s, value: %d", __func__, value);

	/* OLD -> NEW*/
	//if (!value) {
	//	return (zb_bool_t)nrf_802154_pending_bit_for_addr_set(
	//		(const uint8_t *)addr, (bool)extended);
	//} else {
	//	return (zb_bool_t)nrf_802154_pending_bit_for_addr_clear(
	//		(const uint8_t *)addr, (bool)extended);
	//}
	ret = radio_api->configure(radio_dev,
				   IEEE802154_CONFIG_ACK_FPB,
				   &config);
	return !ret ? ZB_TRUE : ZB_FALSE;
}

void zb_trans_src_match_tbl_drop(void)
{
	struct ieee802154_config config = {
		.ack_fpb = {
			.addr = NULL,
			.enabled = false
		}
	};

	LOG_DBG("Function: %s", __func__);

	/* reset for short addresses */
	/* OLD -> NEW*/
	//nrf_802154_pending_bit_for_addr_reset(ZB_FALSE);
	config.ack_fpb.extended = false;
	radio_api->configure(radio_dev, IEEE802154_CONFIG_ACK_FPB, &config);

	/* reset for long addresses */
	/* OLD -> NEW*/
	//nrf_802154_pending_bit_for_addr_reset(ZB_TRUE);
	config.ack_fpb.extended = true;
	radio_api->configure(radio_dev, IEEE802154_CONFIG_ACK_FPB, &config);
}

#ifdef CONFIG_RADIO_STATISTICS
static zb_osif_radio_stats_t nrf_radio_statistics;

zb_osif_radio_stats_t *zb_osif_get_radio_stats(void)
{
	return &nrf_radio_statistics;
}
#endif /* defined CONFIG_RADIO_STATISTICS */

zb_time_t osif_sub_trans_timer(zb_time_t t2, zb_time_t t1)
{
	return ZB_TIME_SUBTRACT(t2, t1);
}

zb_uint8_t zb_trans_get_next_packet(zb_bufid_t buf)
{
	LOG_DBG("Function: %s", __func__);
	zb_uint8_t *data_ptr;
	zb_uint8_t length = 0;

	if (!buf) {
		return 0;
	}

	/*Packed received with correct CRC, PANID and address*/
	struct rx_fifo_item *item = k_fifo_get(&rx_fifo, K_NO_WAIT);

	if (!item) {
		return 0;
	}

	//struct net_pkt *rx_pkt = item->pkt;

	__ASSERT_NO_MSG(item->pkt);

	//length = item->pkt->data[0];
	length = net_pkt_get_len(item->pkt);

	data_ptr = zb_buf_initial_alloc(buf, length);

	/*Copy received data*/
	//ZB_MEMCPY(data_ptr, (void const *)(rx_pkt->data + 1), length);
	net_pkt_cursor_init(item->pkt);
	net_pkt_read(item->pkt, data_ptr, length);

	/*Put LQI, RSSI*/
	zb_macll_metadata_t *metadata = ZB_MACLL_GET_METADATA(buf);

	metadata->lqi = net_pkt_ieee802154_lqi(item->pkt);
	metadata->power = net_pkt_ieee802154_rssi(item->pkt);
	/* Put timestamp (usec) into the packet tail */
	*ZB_BUF_GET_PARAM(buf, zb_time_t) =
		net_pkt_timestamp(item->pkt)->second * USEC_PER_SEC +
		net_pkt_timestamp(item->pkt)->nanosecond / NSEC_PER_USEC;
	/* Additional buffer status for Data Request command */
	zb_macll_set_received_data_status(buf,
		net_pkt_ieee802154_ack_fpb(item->pkt));

	//nrf_802154_buffer_free_raw(rx_pkt->data);
	net_pkt_unref(item->pkt);
	item->pkt = NULL;

	return 1;
}

/************ 802.15.4 radio driver callbacks **************/

/**
 * @brief Notify that frame was transmitted.
 *
 * @note If ACK was requested for transmitted frame this function
 *       is called after proper ACK is received.
 *       If ACK was not requested this function is called just
 *       after transmission is ended.
 * @note Buffer pointed by the @p ack pointer is not modified
 *       by the radio driver (and can't be used to receive a frame)
 *       until @sa nrf_802154_buffer_free_raw() function is called.
 * @note Buffer pointed by the @p ack pointer may be modified by
 *       the function handler (and other modules) until
 *       @sa nrf_802154_buffer_free_raw() function is called.
 * @note The next higher layer should handle @sa nrf_802154_transmitted_raw()
 *       or @sa nrf_802154_transmitted() function. It should not handle both.
 *
 * @param[in]  ack    Pointer to received ACK buffer. Fist byte
 *                    in the buffer is length of the frame (PHR) and
 *                    following bytes are the ACK frame itself (PSDU).
 *                    Length byte (PHR) includes FCS. FCS is already
 *                    verified by the hardware and may be modified by
 *                    the hardware. If ACK was not requested @p ack
 *                    is set to NULL.
 * @param[in]  power  RSSI of received frame or 0 if ACK was not requested.
 * @param[in]  lqi    LQI of received frame or 0 if ACK was not requested.
 */

/*OLD; moved to zb_trans_transmit & ieee802154_radio_handle_ack */
#if 0
void nrf_802154_transmitted_raw(const uint8_t *frame, uint8_t *ack,
				int8_t power, uint8_t lqi)
{
	ARG_UNUSED(frame);
	ARG_UNUSED(power);
	ARG_UNUSED(lqi);

#ifdef CONFIG_RADIO_STATISTICS
	zb_osif_get_radio_stats()->tx_successful++;
#endif /* defined CONFIG_RADIO_STATISTICS */

	zb_macll_transmitted_raw(ack);

	/* Raise signal to indicate radio event */
	zigbee_event_notify(ZIGBEE_EVENT_TX_DONE);
}
#endif

/**
 * @brief Notify that frame was not transmitted due to busy channel.
 *
 * This function is called if transmission procedure fails.
 *
 * @param[in]  error  Reason of the failure.
 */

/* OLD, it's in ieee802154_nrf5.c */
#if 0
void nrf_802154_transmit_failed(uint8_t const *frame,
				nrf_802154_tx_error_t error)
{
	ARG_UNUSED(frame);
#ifdef CONFIG_RADIO_STATISTICS
	switch (error) {
	case NRF_802154_TX_ERROR_NONE:
		zb_osif_get_radio_stats()->tx_err_none++;
		break;
	case NRF_802154_TX_ERROR_BUSY_CHANNEL:
		zb_osif_get_radio_stats()->tx_err_busy_channel++;
		break;
	case NRF_802154_TX_ERROR_INVALID_ACK:
		zb_osif_get_radio_stats()->tx_err_invalid_ack++;
		break;
	case NRF_802154_TX_ERROR_NO_MEM:
		zb_osif_get_radio_stats()->tx_err_no_mem++;
		break;
	case NRF_802154_TX_ERROR_TIMESLOT_ENDED:
		zb_osif_get_radio_stats()->tx_err_timeslot_ended++;
		break;
	case NRF_802154_TX_ERROR_NO_ACK:
		zb_osif_get_radio_stats()->tx_err_no_ack++;
		break;
	case NRF_802154_TX_ERROR_ABORTED:
		zb_osif_get_radio_stats()->tx_err_aborted++;
		break;
	case NRF_802154_TX_ERROR_TIMESLOT_DENIED:
		zb_osif_get_radio_stats()->tx_err_timeslot_denied++;
		break;
	}
#endif /* defined CONFIG_RADIO_STATISTICS */

	switch (error) {
	case NRF_802154_TX_ERROR_NO_MEM:
	case NRF_802154_TX_ERROR_ABORTED:
	case NRF_802154_TX_ERROR_TIMESLOT_DENIED:
	case NRF_802154_TX_ERROR_TIMESLOT_ENDED:
	case NRF_802154_TX_ERROR_BUSY_CHANNEL:
		zb_macll_transmit_failed(ZB_TRANS_CHANNEL_BUSY_ERROR);
		break;

	case NRF_802154_TX_ERROR_INVALID_ACK:
	case NRF_802154_TX_ERROR_NO_ACK:
		zb_macll_transmit_failed(ZB_TRANS_NO_ACK);
		break;
	}

	/* Raise signal to indicate radio event */
	zigbee_event_notify(ZIGBEE_EVENT_TX_FAILED);
}
#endif


/*OLD; it's done in ieee802154_nrf5.c*/
#if 0
/* Callback notifies about the start of the ACK frame transmission. */
void nrf_802154_tx_ack_started(const uint8_t *data)
{
    /* Check if the frame pending bit is set in ACK frame. */
	acked_with_pending_bit = data[FRAME_PENDING_OFFSET] & FRAME_PENDING_BIT;
}
#endif

/**
 * @brief Notify that frame was received.
 *
 * @note Buffer pointed by the data pointer is not modified
 *       by the radio driver (and can't be used to receive a frame)
 *       until nrf_802154_buffer_free_raw() function is called.
 * @note Buffer pointed by the data pointer may be modified by the function
 *       handler (and other modules) until @sa nrf_802154_buffer_free_raw()
 *       function is called.
 * @note The next higher layer should handle @sa nrf_802154_received_raw()
 *       or @sa nrf_802154_received() function. It should not handle both.
 *
 * data
 * v
 * +-----+---------------------------------------------------+------------+
 * | PHR | MAC Header and payload                            | FCS        |
 * +-----+---------------------------------------------------+------------+
 *       |                                                                |
 *       | <---------------------------- PHR ---------------------------> |
 *
 * @param[in]  data    Pointer to the buffer containing received data
 *                     (PHR + PSDU). First byte in the buffer is length
 *                     of the frame (PHR) and following bytes is the frame
 *                     itself (PSDU). Length byte (PHR) includes FCS.
 *                     FCS is already verified by the hardware and may be
 *                     modified by the hardware.
 * @param[in]  power   RSSI of received frame.
 * @param[in]  lqi     LQI of received frame.
 */

/*OLD; replaced with zigbee_l2_recv*/
#if 0
void nrf_802154_received_timestamp_raw(uint8_t *data, int8_t power,
				       uint8_t lqi, uint32_t time)
{
#ifdef CONFIG_RADIO_STATISTICS
	zb_osif_get_radio_stats()->rx_successful++;
#endif /* defined CONFIG_RADIO_STATISTICS */

	rx_frame_t *rx_frame_free_slot = NULL;

	/* Find free slot for frame */
	for (uint32_t i = 0; i < ARRAY_SIZE(rx_frames); i++) {
		if (rx_frames[i].data == NULL) {
			rx_frame_free_slot = &rx_frames[i];
			break;
		}
	}

	if (rx_frame_free_slot == NULL) {
		__ASSERT(false,
			 "Not enough rx frames allocated for 15.4 driver");
		return;
	}

	rx_frame_free_slot->data = data;
	rx_frame_free_slot->power = power;
	rx_frame_free_slot->lqi = lqi;
	rx_frame_free_slot->time = time;

	if (data[ACK_REQUEST_OFFSET] & ACK_REQUEST_BIT) {
		rx_frame_free_slot->pending_bit = acked_with_pending_bit;
	} else {
		rx_frame_free_slot->pending_bit = ZB_FALSE;
	}

	acked_with_pending_bit = ZB_FALSE;
	k_fifo_put(&rx_fifo, rx_frame_free_slot);

	zb_macll_set_rx_flag();
	zb_macll_set_trans_int();

	/* Raise signal to indicate radio event */
	zigbee_event_notify(ZIGBEE_EVENT_RX_DONE);
}
#endif

/*OLD; it's done in ieee802154_nrf5.c*/
#if 0
/* Callback notify that reception of a frame failed. */
void nrf_802154_receive_failed(nrf_802154_rx_error_t error)
{
	acked_with_pending_bit = ZB_FALSE;

#ifdef CONFIG_RADIO_STATISTICS
	switch (error) {
	case NRF_802154_RX_ERROR_NONE:
		zb_osif_get_radio_stats()->rx_err_none++;
		break;
	case NRF_802154_RX_ERROR_INVALID_FRAME:
		zb_osif_get_radio_stats()->rx_err_invalid_frame++;
		break;
	case NRF_802154_RX_ERROR_INVALID_FCS:
		zb_osif_get_radio_stats()->rx_err_invalid_fcs++;
		break;
	case NRF_802154_RX_ERROR_INVALID_DEST_ADDR:
		zb_osif_get_radio_stats()->rx_err_invalid_dest_addr++;
		break;
	case NRF_802154_RX_ERROR_RUNTIME:
		zb_osif_get_radio_stats()->rx_err_runtime++;
		break;
	case NRF_802154_RX_ERROR_TIMESLOT_ENDED:
		zb_osif_get_radio_stats()->rx_err_timeslot_ended++;
		break;
	case NRF_802154_RX_ERROR_ABORTED:
		zb_osif_get_radio_stats()->rx_err_aborted++;
		break;
	}
#endif /* defined CONFIG_RADIO_STATISTICS */
}
#endif

/*OLD; replaced with shim callback*/
#if 0
/* Callback notify that Energy Detection procedure finished. */
void nrf_802154_energy_detected(uint8_t result)
{
	energy_detect.rssi_val = result;
	k_sem_give(&energy_detect.sem);
}

/* Callback notifies that the energy detection procedure failed. */
void nrf_802154_energy_detection_failed(nrf_802154_ed_error_t error)
{
	ARG_UNUSED(error);
	energy_detect.failed = true;
	k_sem_give(&energy_detect.sem);
}
#endif

/* L2 temporarily here*/

void ieee802154_init(struct net_if *iface)
{
	//ARG_UNUSED(iface);

	//struct ieee802154_context *ctx = net_if_l2_data(iface);
	//net_if_up(iface);
	//net_if_flag_set(iface, NET_IF_UP);

	__ASSERT_NO_MSG(radio_dev != NULL);
	__ASSERT_NO_MSG(iface != NULL);

	radio_if = iface;

	k_fifo_init(&rx_fifo);
	k_sem_init(&energy_detect.sem, 1, 1);

	if (IS_ENABLED(CONFIG_IEEE802154_NET_IF_NO_AUTO_START)) {
		LOG_DBG("Interface auto start disabled.");
		net_if_flag_set(iface, NET_IF_NO_AUTO_START);
		return;
	}

	/* copy from zb_trans_hw_init */
	/* OLD -> NEW */
	//(void)nrf_802154_receive();
	radio_api->start(radio_dev);
	state_cache.radio_state = RADIO_802154_STATE_RECEIVE;

	/* OLD; shim configures irq thru driver macro? */
	//nrf5_irq_config();

	net_if_up(iface);
	LOG_DBG("The 802.15.4 interface initialized.");

	/* end copy from zb_trans_hw_init */

//#if CONFIG_IEEE802154_NET_IF_NO_AUTO_START
}

enum net_verdict ieee802154_radio_handle_ack(struct net_if *iface,
					     struct net_pkt *pkt)
{
	ARG_UNUSED(iface);

	size_t ack_len = net_pkt_get_len(pkt);

	if (ack_len != ACK_PKT_LENGTH) {
		LOG_ERR("%s: nieprawidlowa dlugosc ACK", __func__);
		return NET_CONTINUE;
	}

	if ((*net_pkt_data(pkt) & FRAME_TYPE_MASK) != FRAME_TYPE_ACK) {
		LOG_ERR("%s: nieprawidlowy FRAME_TYPE_ACK", __func__);
		return NET_CONTINUE;
	}

	if (ack_frame != NULL) {
		LOG_ERR("Overwriting unhandled ACK frame.");
	}

	ack_frame_buf[0] = ack_len;
	if (net_pkt_read(pkt, &ack_frame_buf[1], ack_len) < 0) {
		LOG_ERR("Failed to read ACK frame.");
		return NET_CONTINUE;
	}

	/* ack_frame != NULL means that ack frame was received */
	ack_frame = ack_frame_buf;

	return NET_OK;
}

static enum net_verdict zigbee_l2_recv(struct net_if *iface,
					struct net_pkt *pkt)
{
#ifdef CONFIG_RADIO_STATISTICS
	zb_osif_get_radio_stats()->rx_successful++;
#endif /* defined CONFIG_RADIO_STATISTICS */

	struct rx_fifo_item *free_slot = NULL;

	/* Find free slot for frame */
	for (uint32_t i = 0; i < ARRAY_SIZE(rx_fifo_buf); i++) {
		if (rx_fifo_buf[i].pkt == NULL) {
			free_slot = &rx_fifo_buf[i];
			break;
		}
	}

	if (free_slot == NULL) {
		__ASSERT(false,
			 "Not enough rx frames allocated for 15.4 driver");
		return NET_DROP;
	}

	//rx_frame_free_slot->data = data;
	//rx_frame_free_slot->power = net_pkt_ieee802154_rssi(pkt);
	//rx_frame_free_slot->lqi = net_pkt_ieee802154_lqi(pkt);
	//rx_frame_free_slot->time = time;
	//rx_frame_free_slot->pending_bit = net_pkt_ieee802154_ack_fpb(pkt);

	//if (data[ACK_REQUEST_OFFSET] & ACK_REQUEST_BIT) {
	//	rx_frame_free_slot->pending_bit = acked_with_pending_bit;
	//} else {
	//	rx_frame_free_slot->pending_bit = ZB_FALSE;
	//}

	free_slot->pkt = pkt;

	//acked_with_pending_bit = ZB_FALSE;
	k_fifo_put(&rx_fifo, free_slot);

	zb_macll_set_rx_flag();
	zb_macll_set_trans_int();

	/* Raise signal to indicate radio event */
	zigbee_event_notify(ZIGBEE_EVENT_RX_DONE);

	return NET_OK;
}

#if 0
static int zigbee_l2_send(struct net_if *iface, struct net_pkt *pkt)
{
	return -ENOENT;
}

static int zigbee_l2_enable(struct net_if *iface, bool state)
{
	return -ENOENT;
}
#endif

static enum net_l2_flags zigbee_l2_flags(struct net_if *iface)
{
	return NET_L2_MULTICAST;
}

NET_L2_INIT(ZIGBEE_L2, zigbee_l2_recv, NULL, NULL /*zigbee_l2_enable*/,
	    zigbee_l2_flags);
