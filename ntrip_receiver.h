/*
 * ntrip_receiver.h
 *
 *  Created on: 13 Aug 2024
 *      Author: jorda
 *
 * Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
 * including the software is for testing purposes only and,
 * because it has limited functions and limited resilience, is not suitable
 * for permanent use under real conditions. If the evaluation board is
 * nevertheless used under real conditions, this is done at one’s responsibility;
 * any liability of Rutronik is insofar excluded
 */

#ifndef NTRIP_RECEIVER_H_
#define NTRIP_RECEIVER_H_

#include <stdint.h>

/**
 * @brief Define the function to be implemented to be informed when a new RTCM packet is available
 */
typedef void (*ntrip_receiver_on_new_rtcm_packet)(uint8_t* buffer, uint16_t len);

typedef enum {
	NTRIP_RECEIVER_STATUS_WAITING_INTERNET_CONNECTION,
	NTRIP_RECEIVER_STATUS_CONNECTED_TO_INTERNET,
	NTRIP_RECEIVER_STATUS_RECEIVING_DATA,
	NTRIP_RECEIVER_STATUS_ERROR
} ntrip_receiver_status_e;

/**
 * @brief Initialize the ntrip receiver module
 */
int ntrip_receiver_init(ntrip_receiver_on_new_rtcm_packet listener);

/**
 * @brief Open connection to a NTRIP server (asynchronuously)
 *
 * Remark: do not support login/password in this version
 */
void ntrip_receiver_open_async(char* ntrip_address, uint16_t ntrip_port, char* ntrip_mount_point);

void ntrip_receiver_stop_async();

/**
 * @brief Cyclic call enabling to check if a new packet is available
 */
ntrip_receiver_status_e ntrip_receiver_do();

#endif /* NTRIP_RECEIVER_H_ */
