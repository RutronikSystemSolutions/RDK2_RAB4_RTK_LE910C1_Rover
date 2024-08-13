/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RutDevKit-PSoC62 Hello World
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2022-10-28
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "rab_rtk.h"
#include "hal_timer.h"

#include "ntrip_receiver.h"


/*Priority for button interrupts*/
#define BTN_IRQ_PRIORITY		5


/**
 * @var request_stop
 * Communication variable between ISR and main process
 * When set to 1, stop of the Telit module is requested
 */
static uint8_t request_stop = 0;
static uint8_t request_start = 0;

#define BUFFER_SIZE 1024
uint8_t buffer[BUFFER_SIZE] = {0};

void on_rtcm_packet(uint8_t* buffer, uint16_t len)
{
	printf("on_rtcm_packet. Len = %d\r\n", len);
}


/*Function prototypes used for this demo.*/
void handle_error(void);
void btn1_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
void btn2_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);

cyhal_gpio_callback_data_t btn1_data =
{
		.callback = btn1_interrupt_handler,
		.callback_arg = NULL,

};

cyhal_gpio_callback_data_t btn2_data =
{
		.callback = btn2_interrupt_handler,
		.callback_arg = NULL,
};

static void do_telit_stuff_1()
{
	telit_pin_status_e pin_status = PIN_STATUS_UNKNOWN;

	int retval = rab_rtk_telit_enable_disable_error_codes(2);
	if (retval != 0)
	{
		printf("rab_rtk_telit_enable_disable_error_codes error: %d\r\n", retval);
		return;
	}

	retval = rab_rtk_telit_get_pin_status(&pin_status);
	if (retval != 0)
	{
		printf("rab_rtk_telit_get_pin_status error: %d\r\n", retval);
		return;
	}

	printf("PIN status: %d \r\n", pin_status);
	if (pin_status != PIN_STATUS_READY)
	{
		printf("PIN not ready, stop\r\n");
		return;
	}

    // Get the attached operator, if not attached, exit
    // it will also gives the network type (GSM or other)
    telit_access_technology_e access_technology = ACCESS_TECHNOLOGY_UNKNOWN;
    retval = rab_rtk_telit_get_attached_operator(&access_technology);
    if (retval != 0)
    {
        printf("rab_rtk_telit_get_attached_operator error: %d\r\n", retval);
        return;
    }
    printf("Access technology: %d \r\n", access_technology);

    telit_registration_status_e registration_status = REGISTRATION_STATUS_UNKNOWN;
    retval = rab_rtk_telit_get_registration_status(&registration_status);
    if (retval != 0)
    {
        printf("rab_rtk_telit_get_registration_status error: %d\r\n", retval);
        return;
    }

    if (registration_status == REGISTRATION_STATUS_REGISTERED)
    {
    	printf("Registration status: REGISTRATION_STATUS_REGISTERED\r\n");
    }
    else if (registration_status == REGISTRATION_STATUS_ROAMING)
    {
    	printf("Registration status: REGISTRATION_STATUS_ROAMING\r\n");
    }
    else
    {
    	printf("Not registered to network, stop\r\n");
    	return;
    }

    retval = rab_rtk_telit_get_gprs_registration_status();
    if (retval != 0)
    {
        printf("rab_rtk_telit_get_gprs_registration_status error: %d\r\n", retval);
        return;
    }

    pdp_context_status_result_t pdp_context_status;
    retval = rab_rtk_telit_get_pdp_context_status(&pdp_context_status);
    if (retval != 0)
    {
        printf("rab_rtk_telit_get_pdp_context_status error: %d\r\n", retval);
        return;
    }

    if (pdp_context_status.status == 0)
    {
    	printf("PDP not activated\r\n");
    }
    else
    {
    	printf("PDP is activated\r\n");
    }

    retval = rab_rtk_telit_get_supported_pdp_contexts();
	if (retval != 0)
	{
		printf("rab_rtk_telit_get_supported_pdp_contexts error: %d\r\n", retval);
		return;
	}

    if (pdp_context_status.status == 0)
    {
    	retval = rab_rtk_telit_define_pdp_context();
		if (retval != 0)
		{
			printf("rab_rtk_telit_define_pdp_context error: %d\r\n", retval);
			return;
		}

        // Only activate the context if needed
        retval = rab_rtk_telit_activate_deactivate_pdp_context(1, 1);
    }
    else
    {
        retval = rab_rtk_telit_activate_deactivate_pdp_context(1, 0);
        printf("PDP context deactivated now, return \r\n");
        return;
    }

    if (retval != 0)
    {
        printf("rab_rtk_telit_activate_deactivate_pdp_context error: %d\r\n", retval);
        return;
    }



    // Try to open a socket with the server
    retval = rab_rtk_telit_open_socket("jordanrutronik.mywire.org", 1507);
    if (retval != 0)
	{
		printf("rab_rtk_telit_open_socket error: %d\r\n", retval);
		return;
	}

    telit_raw_socket_status_e socket_status = SOCKET_STATUS_CLOSED;

    // Wait until connection state is Okay
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
    	return;
    }

    char tosend[]  = "GET /GUIGO HTTP/1.0\r\nUser-Agent: NTRIP RutronikClient/20231025\r\nAccept: */*\r\nConnection: close\r\n\r\n";

    printf("Connection opened - send request %s \r\n", tosend);

    rab_rtk_telit_socket_write(1, strlen(tosend), (uint8_t*)tosend);

    printf("***********************************\r\n");

    for(int i = 0; i < 2; ++i)
	{
    	uint8_t read_buffer[128] = {0};

		rab_rtk_telit_get_socket_status(&socket_status);
		CyDelay(1000);

		uint16_t sent_size = 0;
		uint16_t received_size = 0;
		uint16_t buff_in_size = 0;
		rab_rtk_telit_get_socket_information(&sent_size, &received_size, &buff_in_size);

		if (buff_in_size > 0)
		{
			rab_rtk_telit_socket_read(1, 128, read_buffer);
		}
	}

    rab_rtk_telit_close_socket();
}

int main(void)
{
    cy_rslt_t result;

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif /* #if defined (CY_DEVICE_SECURE) */

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /*Initialize LEDs*/
    result = cyhal_gpio_init( LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Initialize Buttons*/
    result = cyhal_gpio_init(USER_BTN1, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init(USER_BTN2, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Register callback functions */
    cyhal_gpio_register_callback(USER_BTN1, &btn1_data);
    cyhal_gpio_register_callback(USER_BTN2, &btn2_data);

    /* Enable falling edge interrupt events */
    cyhal_gpio_enable_event(USER_BTN1, CYHAL_GPIO_IRQ_FALL, BTN_IRQ_PRIORITY, true);
    cyhal_gpio_enable_event(USER_BTN2, CYHAL_GPIO_IRQ_FALL, BTN_IRQ_PRIORITY, true);

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS) handle_error();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("RDK2 RABRTK LE910C1 Rover\r\n");

//    int retval = rab_rtk_init_gpios();
//    if (retval != 0)
//    {
//        printf("rab_rtk_init_gpios error: %d\r\n", retval);
//        handle_error();
//    }
//
//    retval = hal_timer_init();
//    if (retval != 0)
//    {
//        printf("hal_timer_init error: %d\r\n", retval);
//        handle_error();
//    }
//
//    retval = rab_rtk_init_telit_uart();
//    if (retval != 0)
//    {
//        printf("rab_rtk_init_telit_uart error: %d\r\n", retval);
//        handle_error();
//    }
//
    int retval = ntrip_receiver_init(on_rtcm_packet);
    printf("ntrip_receiver_init returns: %d \r\n", retval);

    //    const uint16_t rPort = 2101;
    //    char ipaddr[] = "caster.centipede.fr";

    //ntrip_receiver_open_async("jordanrutronik.mywire.org", 1507, "GUIGO");
    //ntrip_receiver_open_async("caster.centipede.fr", 2101, "GUIGO");

    for(;;)
    {
    	ntrip_receiver_do();

    	if (request_start)
    	{
    		ntrip_receiver_stop_async();
    	}

//        // Cyclic call to process and do
//        //rab_rtk_do();
//
//        if (request_start)
//        {
//            request_start = 0;
//            printf("Turn ON\r\n");
//            rab_rtk_telit_power_on();
//        }
//
//        if (request_stop)
//        {
//            request_stop = 0;
//            do_telit_stuff_1();
//        }
    }
}

/* Interrupt handler callback function */
void btn1_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
	CY_UNUSED_PARAMETER(handler_arg);
    CY_UNUSED_PARAMETER(event);
    request_start = 1;
}

/* Interrupt handler callback function */
void btn2_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
	CY_UNUSED_PARAMETER(handler_arg);
    CY_UNUSED_PARAMETER(event);
    request_stop = 1;
}

/*If initialization fails, program ends up here.*/
void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/* [] END OF FILE */
