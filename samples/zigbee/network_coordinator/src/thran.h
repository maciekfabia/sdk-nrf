/*
 * thran.h
 *
 *  Created on: 9 lip 2020
 *      Author: mafa
 */
#ifndef NRF_SAMPLES_ZIGBEE_NETWORK_COORDINATOR_SRC_THRAN_H_
#define NRF_SAMPLES_ZIGBEE_NETWORK_COORDINATOR_SRC_THRAN_H_

#include <debug/thread_analyzer.h>
#include <string.h>

void update_thread_item(struct thread_analyzer_info *info);
void list_thread_items(void);

#endif /* NRF_SAMPLES_ZIGBEE_NETWORK_COORDINATOR_SRC_THRAN_H_ */
