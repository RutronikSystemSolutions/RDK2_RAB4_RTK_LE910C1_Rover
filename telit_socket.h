/*
 * telit_socket.h
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

#ifndef TELIT_SOCKET_H_
#define TELIT_SOCKET_H_

#include <stdint.h>

typedef enum
{
	TELIT_SOCKET_STATUS_CHECK_MODEM_STATUS,
	TELIT_SOCKET_STATUS_WAIT_MODEM_ON,
	TELIT_SOCKET_STATUS_INIT_INTERNET_CONNECTION,
	TELIT_SOCKET_STATUS_ERROR,
	TELIT_SOCKET_STATUS_CONNECTED
} telit_socket_status_e;

/**
 * @brief Initialize the module
 */
int telit_socket_init();

telit_socket_status_e telit_socket_do();

int telit_socket_open(char* address, uint16_t port);

void telit_socket_close();

int telit_socket_write(uint16_t buffer_len, uint8_t* buffer);

int telit_socket_read(uint16_t buffer_len, uint8_t* buffer);

int telit_socket_get_buffer_in_size(uint16_t* len);

#endif /* TELIT_SOCKET_H_ */
