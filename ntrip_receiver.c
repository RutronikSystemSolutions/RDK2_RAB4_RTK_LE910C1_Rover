/*
 * ntrip_receiver.c
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

#include "ntrip_receiver.h"
#include "telit_socket.h"

#include <stddef.h>
#include <string.h>

#include <stdio.h>

static ntrip_receiver_on_new_rtcm_packet rtcm_listener = NULL;

static ntrip_receiver_status_e local_status = NTRIP_RECEIVER_STATUS_WAITING_INTERNET_CONNECTION;
static uint8_t connection_pending = 0;
static uint8_t stop_pending = 0;
static char address[64] = {0};
static char mount_point[64] = {0};
static uint16_t port = 0;

#define READ_BUFFER_LEN 128

static uint8_t read_buffer[READ_BUFFER_LEN] = {0};

int ntrip_receiver_init(ntrip_receiver_on_new_rtcm_packet listener)
{
	if (listener == NULL) return -1;
	rtcm_listener = listener;

	int retval = telit_socket_init();
	if (retval != 0) return retval - 10;

	return 0;
}

void ntrip_receiver_open_async(char* ntrip_address, uint16_t ntrip_port, char* ntrip_mount_point)
{
	strcpy(address, ntrip_address);
	strcpy(mount_point, ntrip_mount_point);
	port = ntrip_port;
	connection_pending = 1;
}

void ntrip_receiver_stop_async()
{
	stop_pending = 1;
}

ntrip_receiver_status_e ntrip_receiver_do()
{
	switch(local_status)
	{
	case NTRIP_RECEIVER_STATUS_WAITING_INTERNET_CONNECTION:
		{
			telit_socket_status_e status = telit_socket_do();
			if (status == TELIT_SOCKET_STATUS_CONNECTED)
			{
				// Connected, we are ready to open a socket
				local_status = NTRIP_RECEIVER_STATUS_CONNECTED_TO_INTERNET;
				printf("NTRIP_RECEIVER_STATUS_CONNECTED_TO_INTERNET \r\n");
			}
			else if (status == TELIT_SOCKET_STATUS_ERROR)
			{
				local_status = NTRIP_RECEIVER_STATUS_ERROR;
				printf("NTRIP_RECEIVER_STATUS_ERROR \r\n");
			}
			break;
		}

	case NTRIP_RECEIVER_STATUS_CONNECTED_TO_INTERNET:
		if (connection_pending)
		{
			int retval = telit_socket_open(address, port);
			printf("telit_socket_open: %d \r\n", retval);
			if (retval != 0)
			{
				printf("telit_socket_open error: %d\r\n", retval);
				local_status = NTRIP_RECEIVER_STATUS_ERROR;
			}
			else
			{
				char tosend[256] = {0};
				sprintf(tosend, "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP RutronikClient/20231025\r\nAccept: */*\r\nConnection: close\r\n\r\n", mount_point);
				printf("Connection opened - send request %s \r\n", tosend);
				telit_socket_write(strlen(tosend), (uint8_t*)tosend);

				local_status = NTRIP_RECEIVER_STATUS_RECEIVING_DATA;
			}
		    connection_pending = 0;
		}
		break;

	case NTRIP_RECEIVER_STATUS_RECEIVING_DATA:
		{
			uint16_t buff_in_size = 0;
			int retval = telit_socket_get_buffer_in_size(&buff_in_size);
			if (retval != 0)
			{
				// Happens sometimes because of SRING...
				break;
			}

			if (buff_in_size > 0)
			{
				uint16_t toread = (buff_in_size > READ_BUFFER_LEN) ? READ_BUFFER_LEN : buff_in_size;
				retval = telit_socket_read(toread, read_buffer);
				if (retval != 0)
				{
					printf("telit_socket_read error: %d\r\n", retval);
					local_status = NTRIP_RECEIVER_STATUS_ERROR;
				}
				// printf("Received: %d - buff in: %d \r\n", toread, buff_in_size);
				rtcm_listener(read_buffer, toread);
			}

			if (stop_pending)
			{
				printf("Disconnect\r\n");
				stop_pending = 0;
				local_status = NTRIP_RECEIVER_STATUS_ERROR;
				telit_socket_close();
			}

			break;
		}

	case NTRIP_RECEIVER_STATUS_ERROR:
		break;
	}

	return local_status;
}
