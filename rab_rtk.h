/*
 * rab_rtk.h
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

#ifndef RAB_RTK_H_
#define RAB_RTK_H_

#include <stdint.h>

typedef struct
{
    uint8_t index; /**< PDP index */
    uint8_t status; /**< PDP status - 0 or 1 */
} pdp_context_status_result_t;

typedef enum
{
    PIN_STATUS_READY,
    PIN_STATUS_WAITING_PIN,
    PIN_STATUS_UNKNOWN
} telit_pin_status_e;

typedef enum
{
    ACCESS_TECHNOLOGY_GSM,
    ACCESS_TECHNOLOGY_UTRAN,
    ACCESS_TECHNOLOGY_EUTRAN,
    ACCESS_TECHNOLOGY_UNKNOWN
} telit_access_technology_e;

typedef enum
{
    REGISTRATION_STATUS_NOT_REGISTERED,
    REGISTRATION_STATUS_REGISTERED,
    REGISTRATION_STATUS_SEARCHING_FOR_OPERATOR,
    REGISTRATION_STATUS_DENIED,
    REGISTRATION_STATUS_ROAMING,
    REGISTRATION_STATUS_UNKNOWN
} telit_registration_status_e;

typedef enum
{
    SOCKET_STATUS_CLOSED,
	SOCKET_STATUS_ACTIVE,
	SOCKET_STATUS_SUSPENDED, // Remark: suspended but still opened!
	SOCKET_STATUS_SUSPENDED_WITH_PENDING_DATA,
	SOCKET_STATUS_LISTENING,
	SOCKET_STATUS_INCOMING_CONNECTION,
	SOCKET_STATUS_OPENING_PROCESS
} telit_raw_socket_status_e;


int rab_rtk_telit_init_hardware();


/**
* @brief Initialize the RTK board (GPIOs) to have a defined start
*/
int rab_rtk_init_gpios();

/**
* @brief Initialise the UART communication with the Telit module
*/
int rab_rtk_init_telit_uart();

int rab_rtk_do();

void rab_rtk_telit_power_on();
void rab_rtk_telit_power_off();

int rab_rtk_telit_is_pin_ready();

int rab_rtk_telit_get_pdp_context_status(pdp_context_status_result_t* result);

/**
 * @brief Get the supported PDP context.
 *
 * If no supported PDP context, then it will not be possible to activate a PDP context...
 */
int rab_rtk_telit_get_supported_pdp_contexts();

int rab_rtk_telit_define_pdp_context();

int rab_rtk_telit_set_techno();

int rab_rtk_telit_do_for_support();

int rab_rtk_telit_set_eps_mode_of_operation();

int rab_rtk_telit_get_gprs_registration_status();

int rab_rtk_telit_test();

int rab_rtk_telit_delete_pdp_context(uint8_t id);

int rab_rtk_telit_activate_deactivate_pdp_context(uint16_t context_id, uint16_t status);

int rab_rtk_telit_open_socket(char* address, uint16_t port);

int rab_rtk_telit_close_socket();

int rab_rtk_telit_get_socket_status(telit_raw_socket_status_e* status);

int rab_rtk_telit_get_socket_information(uint16_t* sent_size, uint16_t* received_size, uint16_t* buff_in_size);

/**
 * @brief Enable or disable error codes when calling a command
 *
 * @param [in] flag	0: disable, 1: use numeric error codes, 2: use verbose error codes
 */
int rab_rtk_telit_enable_disable_error_codes(uint8_t flag);

int rab_rtk_telit_get_pin_status(telit_pin_status_e* status);

int rab_rtk_telit_get_attached_operator(telit_access_technology_e* access_technology);

int rab_rtk_telit_get_registration_status(telit_registration_status_e* registration_status);

/**
* @brief Send data over a socket
*
*/
int rab_rtk_telit_socket_write(uint8_t connection_id, uint16_t len, uint8_t* buffer);

int rab_rtk_telit_socket_read(uint8_t connection_id, uint16_t buffer_len, uint8_t* buffer);


#endif // RAB_RTK_H_
