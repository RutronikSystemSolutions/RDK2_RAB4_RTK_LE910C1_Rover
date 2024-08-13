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

#include "hal/hal_uart.h"

#include "um980/um980_app.h"
#include "um980/packet_printer.h"
#include "um980/nmea_packet.h"


/*Priority for button interrupts*/
#define BTN_IRQ_PRIORITY		5

/**
 * @var request_stop
 * Communication variable between ISR and main process
 * When set to 1, disconnect
 */
static uint8_t request_stop = 0;

void on_rtcm_packet(uint8_t* buffer, uint16_t len)
{
	cyhal_gpio_toggle(LED1);
	hal_uart_write(buffer, len);
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

void do_blink()
{
	static const uint32_t timeout = 500000; // 0.5 seconds
	static uint32_t last_toggle = 0;
	uint32_t current = hal_timer_get_uticks();

	if ((current < last_toggle) || ((current - last_toggle) > timeout))
	{
		last_toggle = current;
		cyhal_gpio_toggle(LED2);
	}
}

/**
 * @brief Listener function that will be called when a new NMEA packet is available
 *
 * A typical NMEA packet is a GGA packet containing the position
 */
void um980_nmea_listener(uint8_t* buffer, uint16_t len)
{
	if (nmea_packet_get_type(buffer, len) == PACKET_TYPE_GGA)
	{
		um980_gga_packet_t gga_data;
		if (gga_packet_extract_data(buffer, len, &gga_data) == 0)
		{
			packet_printer_print_gga(&gga_data);
		}
	}
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

    // Initialize UART (for UM980)
	if (hal_uart_init() != 0)
	{
		handle_error();
	}
	printf("UART initialized.\r\n");

    int retval = ntrip_receiver_init(on_rtcm_packet);
    printf("ntrip_receiver_init returns: %d \r\n", retval);

    // Initialize UM980
	um980_app_init_hal(hal_uart_readable, hal_uart_read, hal_uart_write, hal_timer_get_uticks);
	um980_app_set_nmea_listener(um980_nmea_listener);

	// Stop message generation (correction and position)
	// Wait until successful - First call might fail because of strange startup behavior
	for(;;)
	{
		retval = um980_app_unlog();
		if (retval != 0)
		{
			printf("um980_app_unlog error %d ! \r\n", retval);
			Cy_SysLib_Delay(1000);
			um980_app_reset();
		}
		else break;
	}

	if (um980_app_set_mode_rover() != 0)
	{
		printf("um980_app_set_mode_rover error ! \r\n");
		handle_error();
	}

	// Wait until connected to the Internet
	for(;;)
	{
		ntrip_receiver_status_e status = ntrip_receiver_do();
		if (status == NTRIP_RECEIVER_STATUS_CONNECTED_TO_INTERNET)
		{
			printf("Alright, connected, continue\r\n");
			break;
		}
	}

	// Now connected to the Internet
	// Request position every second
	if (um980_app_start_gga_generation(2) != 0)
	{
		printf("um980_app_start_gga_generation error ! \r\n");
		handle_error();
	}

	// Open connection with NTRIP provider
    ntrip_receiver_open_async("caster.centipede.fr", 2101, "GUIGO");

    for(;;)
    {
    	ntrip_receiver_do();

    	if (um980_app_do() != 0)
		{
			printf("Error app_do()\r\n");
			handle_error();
		}

    	if (request_stop)
    	{
    		ntrip_receiver_stop_async();
    	}

    	do_blink();
    }
}

/* Interrupt handler callback function */
void btn1_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
	CY_UNUSED_PARAMETER(handler_arg);
    CY_UNUSED_PARAMETER(event);
    request_stop = 1;
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
	printf("Something wrong happened. \r\n");

	// both LEDs OFF
	cyhal_gpio_write(LED1, CYBSP_LED_STATE_OFF);
	cyhal_gpio_write(LED2, CYBSP_LED_STATE_OFF);

     /* Disable all interrupts. */
    __disable_irq();
    CY_ASSERT(0);
}

/* [] END OF FILE */
