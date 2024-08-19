/*
 * telit_socket.c
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

#include "telit_socket.h"

#include "rab_rtk.h"

#include <stdio.h>

#include "cy_syslib.h"

telit_socket_status_e local_status = TELIT_SOCKET_STATUS_CHECK_MODEM_STATUS;

static int is_modem_on()
{
	telit_pin_status_e pin_status = PIN_STATUS_UNKNOWN;
	int retval = rab_rtk_telit_get_pin_status(&pin_status);
	if (retval != 0)
	{
		// Not on, MODEM does not react
		return 0;
	}
	// Modem is ready
	return 1;
}

static int init_internet_connection()
{
	int retval = rab_rtk_telit_enable_disable_error_codes(2);
	if (retval != 0)
	{
		printf("rab_rtk_telit_enable_disable_error_codes error: %d\r\n", retval);
		return -1;
	}

	telit_pin_status_e pin_status = PIN_STATUS_UNKNOWN;
	retval = rab_rtk_telit_get_pin_status(&pin_status);
	if (retval != 0)
	{
		printf("rab_rtk_telit_get_pin_status error: %d\r\n", retval);
		return -2;
	}

	printf("PIN status: %d \r\n", pin_status);
	if (pin_status != PIN_STATUS_READY)
	{
		printf("PIN not ready, stop\r\n");
		return -3;
	}

	// Need to wait for the attached operator (it might take time....)
	telit_access_technology_e access_technology = ACCESS_TECHNOLOGY_UNKNOWN;
	for(int i = 0; i < 20; ++i)
	{
		// Get the attached operator, if not attached, exit
		// it will also gives the network type (GSM or other)
		retval = rab_rtk_telit_get_attached_operator(&access_technology);
		if (retval != 0)
		{
			CyDelay(1000);
		}
		else break;
	}

	if (access_technology == ACCESS_TECHNOLOGY_UNKNOWN)
	{
		printf("Error unknown technology\r\n");
		return -4;
	}
	printf("Access technology: %d \r\n", access_technology);

	for(int i = 0; i < 10; ++i)
	{
		telit_registration_status_e registration_status = REGISTRATION_STATUS_UNKNOWN;
		retval = rab_rtk_telit_get_registration_status(&registration_status);
		if (retval != 0)
		{
			printf("rab_rtk_telit_get_registration_status error: %d\r\n", retval);
			return -5;
		}

		if (registration_status == REGISTRATION_STATUS_REGISTERED)
		{
			printf("Registration status: REGISTRATION_STATUS_REGISTERED\r\n");
			break;
		}
		else if (registration_status == REGISTRATION_STATUS_ROAMING)
		{
			printf("Registration status: REGISTRATION_STATUS_ROAMING\r\n");
			break;
		}
		else
		{
			printf("Not registered to network, stop\r\n");
			if (i == 9) return -6;
			CyDelay(1000);
		}
	}


	pdp_context_status_result_t pdp_context_status;
	retval = rab_rtk_telit_get_pdp_context_status(&pdp_context_status);
	if (retval != 0)
	{
		printf("rab_rtk_telit_get_pdp_context_status error: %d\r\n", retval);
		return -7;
	}

	if (pdp_context_status.status == 1)
	{
		// if already activated, deactivate it (because else some strange behavior when opening a socket
		printf("PDP activated -> Deactivate it \r\n");
		rab_rtk_telit_activate_deactivate_pdp_context(1, 0);
		return -8;
	}

	retval = rab_rtk_telit_get_pdp_context_status(&pdp_context_status);
	if (retval != 0)
	{
		printf("rab_rtk_telit_get_pdp_context_status error: %d\r\n", retval);
		return -7;
	}

	// Define PDP context and activate PDP context
	retval = rab_rtk_telit_define_pdp_context();
	if (retval != 0)
	{
		printf("rab_rtk_telit_define_pdp_context error: %d\r\n", retval);
		return -9;
	}

	retval = rab_rtk_telit_activate_deactivate_pdp_context(1, 1);
	if (retval != 0)
	{
		printf("rab_rtk_telit_activate_deactivate_pdp_context error: %d\r\n", retval);
		return -10;
	}

	return 0;
}

int telit_socket_init()
{
	int retval = rab_rtk_telit_init_hardware();
	if (retval != 0) return retval;

	return 0;
}

telit_socket_status_e telit_socket_do()
{
	switch(local_status)
	{
	case TELIT_SOCKET_STATUS_CHECK_MODEM_STATUS:
		// TODO: always reboot for safe start -> then wait for 30 seconds
		rab_rtk_telit_power_off();

		rab_rtk_telit_power_on();

		printf("Wait for 30 seconds\r\n");

		for(int i = 0; i < 30; ++i)
		{
			CyDelay(1000);
		}

		printf("Let's go\r\n");

		local_status = TELIT_SOCKET_STATUS_WAIT_MODEM_ON;

//		if (is_modem_on())
//		{
//			// Good -> init context
//			printf("Modem is ON at start\r\n");
//			local_status = TELIT_SOCKET_STATUS_INIT_INTERNET_CONNECTION;
//		}
//		else
//		{
//			printf("Modem is OFF at start\r\n");
//			// Start (duration 2.6 seconds - blocking) and wait
//			rab_rtk_telit_power_on();
//			local_status = TELIT_SOCKET_STATUS_WAIT_MODEM_ON;
//		}
		break;

	case TELIT_SOCKET_STATUS_WAIT_MODEM_ON:
		if (is_modem_on())
		{
			printf("Modem is ON now\r\n");
			local_status = TELIT_SOCKET_STATUS_INIT_INTERNET_CONNECTION;
		}
		break;

	case TELIT_SOCKET_STATUS_INIT_INTERNET_CONNECTION:
		{
			int retval = init_internet_connection();
			if (retval != 0)
			{
				printf("TELIT_SOCKET_STATUS_ERROR\r\n");
				local_status = TELIT_SOCKET_STATUS_ERROR;
			}
			else
			{
				printf("TELIT_SOCKET_STATUS_CONNECTED\r\n");
				local_status = TELIT_SOCKET_STATUS_CONNECTED;
			}
			break;
		}

	case TELIT_SOCKET_STATUS_CONNECTED:
		break;

	case TELIT_SOCKET_STATUS_ERROR:
		break;

	}
	return local_status;
}

int telit_socket_open(char* address, uint16_t port)
{
    // Try to open a socket with the server
    int retval = rab_rtk_telit_open_socket(address, port);
    if (retval != 0)
	{
		printf("rab_rtk_telit_open_socket error: %d\r\n", retval);
		return -1;
	}

    // Wait until connection state is Okay
    telit_raw_socket_status_e socket_status = SOCKET_STATUS_CLOSED;
	for(int i = 0; i < 5; ++i)
	{
		rab_rtk_telit_get_socket_status(&socket_status);
		if (socket_status != SOCKET_STATUS_CLOSED) break;
		CyDelay(500);
	}

	if (socket_status == SOCKET_STATUS_CLOSED)
	{
		printf("Connection closed - close socket and exit. \r\n");
		rab_rtk_telit_close_socket();
		rab_rtk_telit_activate_deactivate_pdp_context(1, 0);
		return -2;
	}

	return 0;
}

void telit_socket_close()
{
	rab_rtk_telit_close_socket();
	rab_rtk_telit_activate_deactivate_pdp_context(1, 0);
}

int telit_socket_write(uint16_t buffer_len, uint8_t* buffer)
{
	return rab_rtk_telit_socket_write(1, buffer_len, buffer);
}

int telit_socket_read(uint16_t buffer_len, uint8_t* buffer)
{
	return rab_rtk_telit_socket_read(1, buffer_len, buffer);
}

int telit_socket_get_buffer_in_size(uint16_t* len)
{
	uint16_t sent_size = 0;
	uint16_t received_size = 0;
	uint16_t buff_in_size = 0;
	int retval = rab_rtk_telit_get_socket_information(&sent_size, &received_size, &buff_in_size);
	if (retval != 0) return -1;

	*len = buff_in_size;

	return 0;
}
