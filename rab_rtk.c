
#include "rab_rtk.h"

#include "cy_syslib.h"
#include "cyhal_gpio.h"
#include "cycfg_pins.h"
#include "cyhal_uart.h"

#include "hal/hal_timer.h"
#include <stdint.h>
#include "strutils.h"
#include <stdlib.h>

/**
* @var telit_uart_obj
* UART handler enabling to communicate with the Telit module
*/
static cyhal_uart_t telit_uart_obj;

/**
* @var rx_done_flag
* Used to read asynchronuously
*/
static volatile uint8_t rx_done_flag = 0;

#define BUFFER_SIZE 128
uint8_t read_buffer[BUFFER_SIZE+1] = {0};
uint16_t read_buffer_el_count = 0;

#define DEBUG_OUTPUT
#ifdef DEBUG_OUTPUT
#include <stdio.h>
#endif

static const uint32_t STANDARD_TIMEOUT_US = 500000;
static const uint32_t TIMEOUT_2SECONDS = 2000000;
static const uint32_t TIMEOUT_100SECONDS = 100000000;

void hal_uart_callback(void *callback_arg, cyhal_uart_event_t event)
{
	if (event & CYHAL_UART_IRQ_RX_DONE)
	{
		rx_done_flag = 1;
	}
}


int rab_rtk_init_gpios()
{
/**
     * SHDN_CTRL -> 200ms high -> Turn OFF the Telit module
     * Init to 0
     */
    cy_rslt_t result = cyhal_gpio_init(SHDN_CTRL, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);
	if (result != CY_RSLT_SUCCESS) return -1;

	/**
	 * ON_OFF_CTRL -> 1s high: Turn ON, 2.5s high: Turn OFF
	 * Init to 0
	 */
	result = cyhal_gpio_init(ON_OFF_CTRL, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);
	if (result != CY_RSLT_SUCCESS) return -2;

	/**
	 * PWR_LTE_N -> 0: Telit module power supply ON, 1: Telit module power supply OFF
	 * Init to 0 -> Power ON
	 */
	result = cyhal_gpio_init(PWR_LTE_N, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);
	if (result != CY_RSLT_SUCCESS) return -3;

	/**
	 * PWR_GNSS_N -> 0: UM980 module power supply ON, 1: UM980 module power supply OFF
	 * Init to 1 -> Power OFF
	 */
	result = cyhal_gpio_init(PWR_GNSS_N, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);
	if (result != CY_RSLT_SUCCESS) return -4;


    return 0;
}

int rab_rtk_init_telit_uart()
{
    const uint8_t intr_priority = 3;

	cyhal_uart_cfg_t uart_config =
	{
	    .data_bits = 8,
	    .stop_bits = 1,
	    .parity = CYHAL_UART_PARITY_NONE,
	    .rx_buffer = NULL,
	    .rx_buffer_size = 0
	};

	cy_rslt_t result = cyhal_uart_init(&telit_uart_obj, PSOC_TX_LTE_RX, PSOC_RX_LTE_TX, PSOC_CTS_LTE_RTS, PSOC_RTS_LTE_CTS, NULL, &uart_config);
	if (result != CY_RSLT_SUCCESS)
	{
		return -1;
	}

	uint32_t baudrate = 115200;
	result = cyhal_uart_set_baud(&telit_uart_obj, baudrate, &baudrate);
	if (result != CY_RSLT_SUCCESS)
	{
		return -2;
	}

    // Add callbacks for asynchronous read (enables to use the internal 256 bytes buffer)
    cyhal_uart_register_callback(&telit_uart_obj, hal_uart_callback, NULL);
	cyhal_uart_enable_event(&telit_uart_obj,
			(cyhal_uart_event_t) (CYHAL_UART_IRQ_RX_DONE),
			intr_priority,
			1);

    return 0;
}

static int hal_uart_read(uint8_t* buffer, uint16_t size)
{
	const uint32_t timeout_us = 100000; // 100ms
	size_t toread = (size_t)size;
	cy_rslt_t result = cyhal_uart_read_async(&telit_uart_obj, buffer, toread);
	if (result != CY_RSLT_SUCCESS) return -1;

	uint32_t start_time = hal_timer_get_uticks();
	for(;;)
	{
		if (rx_done_flag != 0)
		{
			rx_done_flag = 0;
			break;
		}

		// Check for timeout
		uint32_t current_time = hal_timer_get_uticks();
		if (current_time < start_time) return -3;
		if ((current_time - start_time) > timeout_us)
		{
			return -2;
		}
	}
	return (int)toread;
}

void rab_rtk_telit_power_on()
{
    cyhal_gpio_write(ON_OFF_CTRL, 1);
    CyDelay(2600);
    cyhal_gpio_write(ON_OFF_CTRL, 0);
}

void rab_rtk_telit_power_off()
{
    cyhal_gpio_write(ON_OFF_CTRL, 1);
    CyDelay(2600);
    cyhal_gpio_write(ON_OFF_CTRL, 0);
}

int rab_rtk_do()
{
    uint32_t readable = cyhal_uart_readable(&telit_uart_obj);
    if (readable > 0)
    {
        uint16_t toread = (readable > BUFFER_SIZE)? BUFFER_SIZE : readable;
        int retval = hal_uart_read(read_buffer, toread);
        if (retval < 0)
        {
#ifdef DEBUG_OUTPUT
            printf("hal_uart_read error : %d \r\n", retval);
#endif
            return -1;
        }

#ifdef DEBUG_OUTPUT
        printf("toread: %d \r\n", toread);
        read_buffer[toread] = 0;
        printf("Content: %s \r\n", (char*)read_buffer);
#endif
    }

    return 0;
}

static void read_available()
{
    for(;;)
    {
        uint32_t readable = cyhal_uart_readable(&telit_uart_obj);
        if (readable == 0) return;
        for(;;)
        {
            uint16_t toread = (readable > BUFFER_SIZE)? BUFFER_SIZE : readable;
            int retval = hal_uart_read(read_buffer, toread);
            if (retval < 0)
            {
                break;
            }

            readable -= toread;
            if (readable == 0) break;
        }
    }
}

static int send_command_and_wait(char* cmd, uint32_t timeout_us)
{
    read_available();

#ifdef DEBUG_OUTPUT
    printf("CMD: %s\r\n", cmd);
#endif

    // Send the command
    size_t len = strlen(cmd);
    cyhal_uart_write(&telit_uart_obj, cmd, &len);

    read_buffer_el_count = 0;
    uint32_t start_time = hal_timer_get_uticks();
    for(;;)
    {
        uint32_t readable = cyhal_uart_readable(&telit_uart_obj);
        if (readable > 0)
        {
            uint16_t remaining_size = BUFFER_SIZE - read_buffer_el_count;
            uint16_t toread = (readable > remaining_size)? remaining_size : readable;

            // Buffer is full...
            if (toread == 0)
            {
                printf("Buffer is full: %s . Readable: %lu \r\n", read_buffer, readable);
                return -4;
            }

            int retval = hal_uart_read(&read_buffer[read_buffer_el_count], toread);
            if (retval < 0)
            {
                printf("hal_uart_read error\r\n");
                return -1;
            }
            read_buffer_el_count += toread;

            read_buffer[read_buffer_el_count] = 0;

            if (strstr((char*)read_buffer, "OK\r\n"))
            {
                printf("[%s]\r\n", read_buffer);
                return 0;
            }

            if (strstr((char*)read_buffer, "ERROR\r\n"))
            {
                printf("ERROR: [%s]\r\n", read_buffer);
                uint32_t elapsed_time = hal_timer_get_uticks() - start_time;
                printf("After: %d miliseconds \r\n", (int)(elapsed_time / 1000));
                return -1;
            }

            // TODO: make it not dependent of the 1...
            if (strstr((char*)read_buffer, "SRING: 1\r\n"))
            {
                printf("Received SRING: [%s]\r\n", read_buffer);
                // Reset
                read_buffer_el_count = 0;
                read_buffer[read_buffer_el_count] = 0;
            }
        }

        uint32_t current_time = hal_timer_get_uticks();
		if (current_time < start_time)
        {
            printf("strange timestamp\r\n");
            return -3;
        }
		if ((current_time - start_time) > timeout_us)
        {
            printf("timeout. el_count: %d \r\n", read_buffer_el_count);
            printf("[%s]\r\n", read_buffer);
            return -2;
        }
    }
    return 0;
}

static int extract_result(char* result)
{
    // Format is:
    // COMMAND \r\n
    // \r\n
    // RESULT \r\n
    // \r\n
    // OK \r\n
    const uint16_t START_COUNTER_VALUE = 2;

    uint16_t line_return_counter = 0;
    uint16_t start_index = 0;
    uint16_t stop_index = 0;
    for(uint16_t i = 1; (i < read_buffer_el_count) && (i < BUFFER_SIZE); ++i)
    {
        if (read_buffer[i-1] == '\r' && read_buffer[i] == '\n')
        {
            line_return_counter++;
            if (line_return_counter == START_COUNTER_VALUE)
            {
                start_index = i + 1;
            }
            else if (line_return_counter == (START_COUNTER_VALUE + 1))
            {
                stop_index = i - 1;
                uint16_t len = stop_index - start_index;
                // Copy to result
                memcpy(result, &read_buffer[start_index], len);
                return len;
            }
        }
    }
    return -1;
}

int rab_rtk_telit_is_pin_ready()
{
    char cmd[] = "AT+CPIN?\r\n";
    int retval = send_command_and_wait(cmd, 500000);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("send command and wait error\r\n");
#endif
        return -1;
    }

    // Extract the result to our request
    char result[32] = {0};
    extract_result(result);

#ifdef DEBUG_OUTPUT
    printf("Result is [%s] \r\n", result);
#endif

    if (strncmp("+CPIN: READY", result, strlen("+CPIN: READY")) == 0)
    {
#ifdef DEBUG_OUTPUT
        printf("Matches +CPIN: READY!\r\n");
#endif
        return 0;
    }

    return -2;
}

static int extract_pdp_context_status(char* buffer, uint16_t len, uint8_t* index, uint8_t* status)
{
    //  Buffer looks like: #SGACT: 1,0
    // index = 1
    // status = 0

    for(uint16_t i = 0; i < len; ++i)
    {
        if (buffer[i] == ' ')
        {
            // Check if enough space after that
            uint16_t remaining_len = len - (i + 1);
            if (remaining_len == 3)
            {
                // Extract
                *index = buffer[i+1] - '0';
                *status = buffer[i+3] - '0';

#ifdef DEBUG_OUTPUT
                printf("Index = %d - Status = %d\r\n", *index, *status);
#endif
                return 0;
            }
            else
            {
#ifdef DEBUG_OUTPUT
                printf("remaining_len is %d \r\n", remaining_len);
                return -1;
#endif
            }
        }
    }


#ifdef DEBUG_OUTPUT
    printf("extract_pdp_context_status went wrong\r\n");
#endif
    return -2;
}

int rab_rtk_telit_get_pdp_context_status(pdp_context_status_result_t* pdp_result)
{
    char cmd[] = "AT#SGACT?\r\n";
    int retval = send_command_and_wait(cmd, 500000);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("send command and wait error\r\n");
#endif
        return -1;
    }

    // Extract the result to our request
    char result[32] = {0};
    retval = extract_result(result);

    // Result looks like:
    // #SGACT: 1, 0   --> cid (1), status off (0)

    if (retval < 0)
    {
#ifdef DEBUG_OUTPUT
        printf("extract_result error: %d\r\n", retval);
#endif
        return -2;
    }

#ifdef DEBUG_OUTPUT
    printf("extract_result (%d) is [%s] \r\n", retval, result);
#endif

    if (extract_pdp_context_status(result, (uint16_t)retval,
    &(pdp_result->index), &(pdp_result->status)) != 0)
    {
        return -3;
    }

    return 0;
}

int rab_rtk_telit_get_supported_pdp_contexts()
{
    char cmd[] = "AT+CGDCONT?\r\n";
    int retval = send_command_and_wait(cmd, 500000);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("send command and wait error\r\n");
#endif
        return -1;
    }

    // TODO

    return 0;
}

int rab_rtk_telit_define_pdp_context()
{
    char cmd[] = "AT+CGDCONT=1,\"IP\",\"nxt20p.net\"\r\n";
    int retval = send_command_and_wait(cmd, 500000);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("send command and wait error\r\n");
#endif
        return -1;
    }

    // TODO

    return 0;
}

int rab_rtk_telit_set_techno()
{
    //char cmd[] = "AT+WS46=28\r\n";
	char cmd[] = "AT+WS46=25\r\n";
    int retval = send_command_and_wait(cmd, 500000);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("send command and wait error\r\n");
#endif
        return -1;
    }

    // TODO

    return 0;
}

int rab_rtk_telit_do_for_support()
{
	char cmd_creg[] = "AT+CREG?\r\n";
    int retval = send_command_and_wait(cmd_creg, 500000);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("send command and wait error\r\n");
#endif
        return -1;
    }

    char cmd_ccid[] = "AT+ICCID\r\n";
	retval = send_command_and_wait(cmd_ccid, 500000);
	if (retval != 0)
	{
#ifdef DEBUG_OUTPUT
		printf("send command and wait error\r\n");
#endif
		return -1;
	}

    char cmd_cgreg[] = "AT+CGREG?\r\n";
	retval = send_command_and_wait(cmd_cgreg, 500000);
	if (retval != 0)
	{
#ifdef DEBUG_OUTPUT
		printf("send command and wait error\r\n");
#endif
		return -1;
	}

    char cmd_cops[] = "AT+COPS?\r\n";
	retval = send_command_and_wait(cmd_cops, 500000);
	if (retval != 0)
	{
#ifdef DEBUG_OUTPUT
		printf("send command and wait error\r\n");
#endif
		return -1;
	}

    char cmd_cgdcont[] = "AT+CGDCONT?\r\n";
	retval = send_command_and_wait(cmd_cgdcont, 500000);
	if (retval != 0)
	{
#ifdef DEBUG_OUTPUT
		printf("send command and wait error\r\n");
#endif
		return -1;
	}

    char cmd_swpkgv[] = "AT#SWPKGV\r\n";
	retval = send_command_and_wait(cmd_swpkgv, 500000);
	if (retval != 0)
	{
#ifdef DEBUG_OUTPUT
		printf("send command and wait error\r\n");
#endif
		return -1;
	}

    char cmd_fwswitch[] = "AT#FWSWITCH?\r\n";
	retval = send_command_and_wait(cmd_fwswitch, 500000);
	if (retval != 0)
	{
#ifdef DEBUG_OUTPUT
		printf("send command and wait error\r\n");
#endif
		return -1;
	}

    char cmd_cgcontrdp[] = "AT+CGCONTRDP=1\r\n";
	retval = send_command_and_wait(cmd_cgcontrdp, 500000);
	if (retval != 0)
	{
#ifdef DEBUG_OUTPUT
		printf("send command and wait error\r\n");
#endif
		return -1;
	}

    char cmd_ws46[] = "AT+WS46?\r\n";
	retval = send_command_and_wait(cmd_ws46, 500000);
	if (retval != 0)
	{
#ifdef DEBUG_OUTPUT
		printf("send command and wait error\r\n");
#endif
		return -1;
	}

    return 0;
}

int rab_rtk_telit_set_eps_mode_of_operation()
{
    char cmd[] = "AT+CEMODE=2\r\n";
    int retval = send_command_and_wait(cmd, 500000);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("send command and wait error\r\n");
#endif
        return -1;
    }

    // TODO

    return 0;
}


int rab_rtk_telit_get_gprs_registration_status()
{
    char cmd[] = "AT+CGREG?\r\n";
    int retval = send_command_and_wait(cmd, 500000);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("send command and wait error\r\n");
#endif
        return -1;
    }

    // TODO

    return 0;
}

int rab_rtk_telit_test()
{
    //char cmd[] = "AT+CGCONTRDP=1\r\n";
	char cmd[] = "AT+WS46?\r\n";
    int retval = send_command_and_wait(cmd, 500000);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("send command and wait error\r\n");
#endif
        return -1;
    }

    // TODO

    return 0;
}


int rab_rtk_telit_delete_pdp_context(uint8_t id)
{
	char cmd[32] = {0};
    sprintf(cmd, "AT+CGDCONT=%d\r\n", id);
    int retval = send_command_and_wait(cmd, 500000);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("send command and wait error\r\n");
#endif
        return -1;
    }

    // TODO

    return 0;
}


int rab_rtk_telit_activate_deactivate_pdp_context(uint16_t context_id, uint16_t status)
{
	uint32_t timeout = TIMEOUT_2SECONDS;
    char cmd[32] = {0};
    sprintf(cmd, "AT#SGACT=%d,%d\r\n", context_id, status);

    if (status != 0)
    {
    	timeout = TIMEOUT_100SECONDS; // longer timeout when trying to activate
    }

    int retval = send_command_and_wait(cmd, timeout);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("send command and wait error\r\n");
#endif
        return -1;
    }

    // Extract the result to our request
    char result[32] = {0};
    retval = extract_result(result);

    // Result looks like:
    // #SGACT: 100.85.6.176,   --> in case status = 1
    // or
    // OK --> in case status = 0

#ifdef DEBUG_OUTPUT
    printf("Result is [%s] \r\n", result);
#endif

    return 0;
}

int rab_rtk_telit_open_socket()
{
    // AT#SD=<connId>,<txProt>,<rPort>,<IPaddr>[,<closureType>[,<lPort>[,<connMode> [,<txTime>[,<userIpType>]]]]]
    // txProt: 0: TCP, 1: UDP
    // closureType: 0 socket close behaviour for TCP
    // lPort UDP connections local port -> 0
    // connMode: 0: online, 1: command mode

    const uint8_t connId = 1;
    const uint8_t txProt = 0; // TCP
    const uint16_t rPort = 1507;
    char ipaddr[] = "jordanrutronik.mywire.org";


//    const uint16_t rPort = 2101;
//    char ipaddr[] = "caster.centipede.fr";

    const uint8_t closureType = 0;
    const uint8_t lPort = 0;
    const uint8_t connMode = 1; // command mode

    char cmd[64] = {0};
    sprintf(cmd, "AT#SD=%d,%d,%d,%s,%d,%d,%d\r\n", connId, txProt, rPort, ipaddr, closureType, lPort, connMode);

    // Remark: timeout set with AT#SCFG
    int retval = send_command_and_wait(cmd, 150000000);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("send command and wait error\r\n");
#endif
        return -1;
    }

    return 0;
}

int rab_rtk_telit_close_socket()
{
    const uint8_t connId = 1;
    char cmd[32] = {0};

    sprintf(cmd, "AT#SH=%d\r\n", connId);

    int retval = send_command_and_wait(cmd, 500000);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("send command and wait error\r\n");
#endif
        return -1;
    }

    return 0;
}

static int extract_socket_status(char* buffer, uint16_t len, uint8_t* socket_status)
{
    // Buffer looks like: #SS: 1,0 or #SS: 1,2,100.85.111.113,32855,90.11.157.67,1507
    // <connId>, <state>, <localIp>, <localPort>, <remoteIp>, <remotePort>
    for(uint16_t i = 0; i < len; ++i)
    {
        if (buffer[i] == ' ')
        {
            // Check if enough space after that
            uint16_t remaining_len = len - (i + 1);
            if (remaining_len >= 3)
            {
                // Extract
                *socket_status = buffer[i+3] - '0';

#ifdef DEBUG_OUTPUT
                printf("socket_status = %d\r\n", *socket_status);
#endif
                return 0;
            }
            else
            {
#ifdef DEBUG_OUTPUT
                printf("remaining_len too short is %d \r\n", remaining_len);
                return -1;
#endif
            }
        }
    }


#ifdef DEBUG_OUTPUT
    printf("extract_pdp_context_status went wrong\r\n");
#endif
    return -2;
}

int rab_rtk_telit_get_socket_status(telit_socket_status_e* status)
{
    char cmd[] = "AT#SS=1\r\n";

    int retval = send_command_and_wait(cmd, 500000);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("send command and wait error\r\n");
#endif
        return -1;
    }

    // Extract the result to our request
    char result[64] = {0};
    retval = extract_result(result);

    // Result looks like:
    // #SS: 1,0 --> Not connected
    // #SS: 1,2,100.85.111.113,32855,90.11.157.67,1507 --> Connected to a server
    // <connId>, <state>, <localIp>, <localPort>, <remoteIp>, <remotePort>
    // with state:
    // 0: closed
    // 1: socket with an activate data transfer connection
    // 2: socket suspended (but still opened!)
    // 3: socket suspended with pending data
    // 4: socket listening
    // 5: socket with incoming connection waiting for accept or shutdown
    // 6: socket in opening process
    if (retval < 0)
    {
#ifdef DEBUG_OUTPUT
        printf("extract_result error: %d\r\n", retval);
#endif
        return -2;
    }

#ifdef DEBUG_OUTPUT
    printf("extract_result (%d) is [%s] \r\n", retval, result);
#endif

    uint8_t socket_status = 0;
    if (extract_socket_status(result, (uint16_t) retval, &socket_status) != 0)
    {
    	return -3;
    }

    // Convert to enum
    switch(socket_status)
    {
		case 0:
			*status = SOCKET_STATUS_CLOSED;
			break;
		case 1:
			*status = SOCKET_STATUS_ACTIVE;
			break;
		case 2:
			*status = SOCKET_STATUS_SUSPENDED;
			break;
		case 3:
			*status = SOCKET_STATUS_SUSPENDED_WITH_PENDING_DATA;
			break;
		case 4:
			*status = SOCKET_STATUS_LISTENING;
			break;
		case 5:
			*status = SOCKET_STATUS_INCOMING_CONNECTION;
			break;
		case 6:
			*status = SOCKET_STATUS_OPENING_PROCESS;
			break;
		default:
			return -4; // Unknown
    }

    return 0;
}

static int extract_socket_information(char* result, int result_len, uint16_t* sent_size, uint16_t* received_size, uint16_t* buff_in_size)
{
    // Buffer looks like
    // #SI: 1,98,0,5766,0  --> 98 bytes send, 0 bytes read, 5766 bytes inside buffer, 0 bytes sent and not acknowledged

	const int sent_size_index = 1;
	const int received_size_index = 2;
	const int buff_in_size_index = 3;

	char buffer[16] = {0};

	// Sent size
	int sent_size_len = strutils_split_and_extract(result, result_len, ',', sent_size_index, buffer);
	if (sent_size_len < 0)
	{
#ifdef DEBUG_OUTPUT
        printf("sent_size_len error: %d\r\n", sent_size_len);
#endif
        return -1;
	}
	*sent_size = atoi(buffer);

	// Received size
	memset(buffer, 0, 16);
	int received_size_len = strutils_split_and_extract(result, result_len, ',', received_size_index, buffer);
	if (received_size_len < 0)
	{
#ifdef DEBUG_OUTPUT
        printf("received_size_len error: %d\r\n", received_size_len);
#endif
        return -2;
	}
	*received_size = atoi(buffer);

	// Buff in size
	memset(buffer, 0, 16);
	int buff_in_size_len = strutils_split_and_extract(result, result_len, ',', buff_in_size_index, buffer);
	if (buff_in_size_len < 0)
	{
#ifdef DEBUG_OUTPUT
        printf("buff_in_size_len error: %d\r\n", buff_in_size_len);
#endif
        return -3;
	}
	*buff_in_size = atoi(buffer);

#ifdef DEBUG_OUTPUT
	printf("sent_size %d bytes\r\n", *sent_size);
	printf("received_size %d bytes\r\n", *received_size);
	printf("buff_in_size %d bytes\r\n", *buff_in_size);
#endif

	return 0;
}

int rab_rtk_telit_get_socket_information(uint16_t* sent_size, uint16_t* received_size, uint16_t* buff_in_size)
{
    char cmd[] = "AT#SI=1\r\n";

    int retval = send_command_and_wait(cmd, 500000);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("send command and wait error %d \r\n", retval);
#endif
        return -1;
    }

    // Extract the result to our request
    char result[64] = {0};
    retval = extract_result(result);

    // Result looks like:
    // #SI: 1,98,0,5766,0  --> 98 bytes send, 0 bytes read, 5766 bytes inside buffer, 0 bytes sent and not acknowledged
    if (retval < 0)
    {
#ifdef DEBUG_OUTPUT
        printf("extract_result error: %d\r\n", retval);
#endif
        return -2;
    }

#ifdef DEBUG_OUTPUT
    printf("extract_result (%d) is [%s] \r\n", retval, result);
#endif

    retval = extract_socket_information(result, retval, sent_size, received_size, buff_in_size);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("extract_socket_information error %d \r\n", retval);
#endif
        return -3;
    }

	return 0;
}

int rab_rtk_telit_enable_disable_error_codes(uint8_t flag)
{
	// Construct the command to be sent
	char cmd[64] = {0};
	sprintf(cmd, "AT+CMEE=%d\r\n", flag);
	int retval = send_command_and_wait(cmd, STANDARD_TIMEOUT_US);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("send command and wait error\r\n");
#endif
        return -1;
    }

    return 0;
}

int rab_rtk_telit_get_pin_status(telit_pin_status_e* status)
{
    char cmd[] = "AT+CPIN?\r\n";
    int retval = send_command_and_wait(cmd, STANDARD_TIMEOUT_US);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("send command and wait error\r\n");
#endif
        *status = PIN_STATUS_UNKNOWN;
        return -1;
    }

    // Extract the result to our request
    char result[64] = {0};
    retval = extract_result(result);
    if (retval < 0)
    {
#ifdef DEBUG_OUTPUT
        printf("extract_result error: %d\r\n", retval);
#endif
        *status = PIN_STATUS_UNKNOWN;
        return -2;
    }

    // Result looks like:
    // +CPIN: READY  --> Ok, no PIN needed
    // +CPIN: SIM PIN  --> Need to enter PIN
    // Other values are possible (see AT documentation)

#ifdef DEBUG_OUTPUT
    printf("extract_result (%d) is [%s] \r\n", retval, result);
#endif

    if (strstr((char*)result, "+CPIN: READY"))
    {
        *status = PIN_STATUS_READY;
    }
    else if (strstr((char*)result, "+CPIN: SIM PIN"))
    {
        *status = PIN_STATUS_WAITING_PIN;
    }
    else
    {
        *status = PIN_STATUS_UNKNOWN;
    }

    return 0;
}

int rab_rtk_telit_get_attached_operator(telit_access_technology_e* access_technology)
{
    const int access_technology_index = 3;
    char cmd[] = "AT+COPS?\r\n";
    int retval = send_command_and_wait(cmd, STANDARD_TIMEOUT_US);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("send command and wait error\r\n");
#endif
        *access_technology = ACCESS_TECHNOLOGY_UNKNOWN;
        return -1;
    }

    // Extract the result to our request
    char result[64] = {0};
    retval = extract_result(result);

    // Result looks like:
    // +COPS: 0  --> Not registered
    // +COPS: 0,0,"F SFR",7
    // Read command returns current value of <mode>, <format>, <oper> and
    // <AcT> in format <format>. If no operator is selected, <format>, <oper> and <AcT> are omitted.
    // +COPS: <mode>[, <format>, <oper>,< act>]

    if (retval < 0)
    {
#ifdef DEBUG_OUTPUT
        printf("extract_result error: %d\r\n", retval);
#endif
        *access_technology = ACCESS_TECHNOLOGY_UNKNOWN;
        return -2;
    }

#ifdef DEBUG_OUTPUT
    printf("extract_result (%d) is [%s] \r\n", retval, result);
#endif

    int comma_count = strutils_count_char(result, retval, ',');
    if (comma_count == 0)
    {
        // Not registered...
#ifdef DEBUG_OUTPUT
        printf("Not registered \r\n");
#endif
        *access_technology = ACCESS_TECHNOLOGY_UNKNOWN;
        return -3;
    }

    if (comma_count != 3)
    {
        // Format problem
#ifdef DEBUG_OUTPUT
        printf("Format problem \r\n");
#endif
        *access_technology = ACCESS_TECHNOLOGY_UNKNOWN;
        return -4;
    }

    // Extract the access technology type
    char technology_type[16] = {0};
    int tec_type_len = strutils_split_and_extract(result, retval, ',', access_technology_index, technology_type);
    if (tec_type_len < 0)
    {
        // Format problem
#ifdef DEBUG_OUTPUT
        printf("Cannot extract len \r\n");
#endif
        *access_technology = ACCESS_TECHNOLOGY_UNKNOWN;
        return -5;
    }

#ifdef DEBUG_OUTPUT
    printf("Technology type is : [%s] \r\n", technology_type);
#endif

    int as_int = atoi(technology_type);
    switch(as_int)
    {
        case 0:
            *access_technology = ACCESS_TECHNOLOGY_GSM;
            break;
        case 2:
            *access_technology = ACCESS_TECHNOLOGY_UTRAN;
            break;
        case 7:
            *access_technology = ACCESS_TECHNOLOGY_EUTRAN;
            break;
        default:
            *access_technology = ACCESS_TECHNOLOGY_UNKNOWN;
            break;
    }


    // AT+CREG? read <mode> and <stat> parameters
    // Mode:
    // 0: disable the network registration unsolicited result code
    // 1: enable the network registration unsolicited result code, and selects the short format
    // 2: enable the network registration unsolicited result code, and selects the long format (includes the network cell identification data)
    // Stat:
    // 0: not registered, terminal is not currently searching a new operator to register to
    // 1: registered, home network
    // 2: not registered, but terminal is currently searching a new operator to register to
    // 3: registration denied
    // 4: unknown
    // 5: registered, roaming
    //send_command_and_wait("AT+CREG?\r\n", STANDARD_TIMEOUT_US);


    return 0;
}

int rab_rtk_telit_get_registration_status(telit_registration_status_e* registration_status)
{
    const int registration_status_index = 1;
    char cmd[] = "AT+CREG?\r\n";
    int retval = send_command_and_wait(cmd, STANDARD_TIMEOUT_US);
    if (retval != 0)
    {
#ifdef DEBUG_OUTPUT
        printf("send command and wait error\r\n");
#endif
        *registration_status = REGISTRATION_STATUS_UNKNOWN;
        return -1;
    }

    // Extract the result to our request
    char result[64] = {0};
    retval = extract_result(result);

    // Result looks like:
    // TODO

    if (retval < 0)
    {
#ifdef DEBUG_OUTPUT
        printf("extract_result error: %d\r\n", retval);
#endif
        *registration_status = REGISTRATION_STATUS_UNKNOWN;
        return -2;
    }

#ifdef DEBUG_OUTPUT
    printf("extract_result (%d) is [%s] \r\n", retval, result);
#endif

    int comma_count = strutils_count_char(result, retval, ',');
    if (comma_count != 1)
    {
        // Format problem
#ifdef DEBUG_OUTPUT
        printf("Format problem \r\n");
#endif
        *registration_status = REGISTRATION_STATUS_UNKNOWN;
        return -4;
    }

    // Extract the access technology type
    char registration_status_str[16] = {0};
    int reg_status_len = strutils_split_and_extract(result, retval, ',', registration_status_index, registration_status_str);
    if (reg_status_len < 0)
    {
        // Format problem
#ifdef DEBUG_OUTPUT
        printf("Cannot extract len \r\n");
#endif
        *registration_status = REGISTRATION_STATUS_UNKNOWN;
        return -5;
    }

#ifdef DEBUG_OUTPUT
    printf("Registration status is : [%s] \r\n", registration_status_str);
#endif

    int as_int = atoi(registration_status_str);
    switch(as_int)
    {
        case 0:
            *registration_status = REGISTRATION_STATUS_NOT_REGISTERED;
            break;
        case 1:
            *registration_status = REGISTRATION_STATUS_REGISTERED;
            break;
        case 2:
            *registration_status = REGISTRATION_STATUS_SEARCHING_FOR_OPERATOR;
            break;
        case 3:
            *registration_status = REGISTRATION_STATUS_DENIED;
            break;
        case 5:
            *registration_status = REGISTRATION_STATUS_ROAMING;
            break;
        default:
            *registration_status = REGISTRATION_STATUS_UNKNOWN;
            break;
    }

    return 0;
}

static int wait_for_socket_input()
{
    uint32_t timeout_us = 10000;
    uint32_t start_time = hal_timer_get_uticks();
    for(;;)
    {
        uint32_t readable = cyhal_uart_readable(&telit_uart_obj);
        if (readable > 0)
        {
            uint16_t remaining_size = BUFFER_SIZE;
            uint16_t toread = (readable > remaining_size)? remaining_size : readable;

            int retval = hal_uart_read(read_buffer, toread);
            if (retval < 0)
            {
                printf("hal_uart_read error\r\n");
                return -1;
            }

            read_buffer[toread] = 0;

            if (strstr((char*)read_buffer, ">"))
            {
                printf("[%s]\r\n", read_buffer);
                uint32_t elapsed_time = hal_timer_get_uticks() - start_time;
                printf("After: %d miliseconds \r\n", (int)(elapsed_time / 1000));
                return 0;
            }

            if (strstr((char*)read_buffer, "ERROR\r\n"))
            {
                printf("ERROR: [%s]\r\n", read_buffer);
                uint32_t elapsed_time = hal_timer_get_uticks() - start_time;
                printf("After: %d miliseconds \r\n", (int)(elapsed_time / 1000));
                return -1;
            }
        }

        uint32_t current_time = hal_timer_get_uticks();
		if (current_time < start_time)
        {
            printf("strange timestamp\r\n");
            return -3;
        }
		if ((current_time - start_time) > timeout_us)
        {
            printf("timeout. el_count: %d \r\n", read_buffer_el_count);
            printf("[%s]\r\n", read_buffer);
            return -2;
        }
    }

    // Should never happen
    return -4;
}

/**
 * @brief Send a command (to receive data from socket) and wait until bytes have been received and OK has been received
 */
static int send_command_and_wait_rcv(char* cmd, uint16_t buffer_len, uint8_t* buffer, uint32_t timeout_us)
{
    read_available();

#ifdef DEBUG_OUTPUT
    printf("CMD: %s\r\n", cmd);
#endif

    // Send the command
    size_t len = strlen(cmd);
    cyhal_uart_write(&telit_uart_obj, cmd, &len);

    uint32_t start_time = hal_timer_get_uticks();

    // Wait for header
    // AT#SRECV=<connId>,<size>\r\n\r\n#SRECV: <connId>,<size>\r\n
    const uint16_t header_size = (uint16_t)len + 2 + (uint16_t)len - 1;
    read_buffer_el_count = 0;
    uint16_t header_to_read = header_size;
	for(;;)
	{
		uint32_t readable = cyhal_uart_readable(&telit_uart_obj);
		if (readable > 0)
		{
			uint16_t toread = (readable > header_to_read)? header_to_read : readable;

			// Buffer is full...
			if (toread == 0)
			{
				printf("Strange! \r\n");
				return -4;
			}

			int retval = hal_uart_read(&read_buffer[read_buffer_el_count], toread);
			if (retval < 0)
			{
				printf("hal_uart_read error\r\n");
				return -1;
			}
			read_buffer_el_count += toread;
			header_to_read -= toread;
			read_buffer[read_buffer_el_count] = 0;

			if (header_to_read == 0)
			{
				printf("Got header: %s \r\n", read_buffer);
				break;
			}

			// TODO set it back
			/*if (strstr((char*)read_buffer, "ERROR\r\n"))
			{
				printf("ERROR: [%s]\r\n", read_buffer);
				uint32_t elapsed_time = hal_timer_get_uticks() - start_time;
				printf("After: %d miliseconds \r\n", (int)(elapsed_time / 1000));
				return -1;
			}*/
		}

		uint32_t current_time = hal_timer_get_uticks();
		if (current_time < start_time)
		{
			printf("strange timestamp\r\n");
			return -3;
		}
		if ((current_time - start_time) > timeout_us)
		{
			printf("timeout. el_count: %d \r\n", read_buffer_el_count);
			printf("[%s]\r\n", read_buffer);
			return -2;
		}
	}

    // Fill the result buffer
    uint16_t toread = buffer_len;
    uint16_t read_address = 0;

    for(;;)
    {
    	if (toread == 0)
		{
    		printf("Ok got enough\r\n");
    		break;
		}

    	uint32_t readable = cyhal_uart_readable(&telit_uart_obj);
    	if (readable > 0)
    	{
    		uint16_t chunk_size = readable;
    		if (toread < readable) chunk_size = toread;

            int retval = hal_uart_read(&buffer[read_address], chunk_size);
            if (retval < 0)
            {
                printf("hal_uart_read error\r\n");
                return -1;
            }

            read_address += chunk_size;
            toread -= chunk_size;
    	}

        uint32_t current_time = hal_timer_get_uticks();
		if (current_time < start_time)
        {
            printf("strange timestamp\r\n");
            return -3;
        }
		if ((current_time - start_time) > timeout_us)
        {
            printf("timeout. toread: %d read_address %d \r\n", toread, read_address);
            return -2;
        }
    }

    read_buffer_el_count = 0;
    // Now wait for OK
    for(;;)
    {
        uint32_t readable = cyhal_uart_readable(&telit_uart_obj);
        if (readable > 0)
        {
            uint16_t remaining_size = BUFFER_SIZE - read_buffer_el_count;
            uint16_t toread = (readable > remaining_size)? remaining_size : readable;

            // Buffer is full...
            if (toread == 0)
            {
                printf("Buffer is full: %s . Readable: %lu \r\n", read_buffer, readable);
                return -4;
            }

            int retval = hal_uart_read(&read_buffer[read_buffer_el_count], toread);
            if (retval < 0)
            {
                printf("hal_uart_read error\r\n");
                return -1;
            }
            read_buffer_el_count += toread;

            read_buffer[read_buffer_el_count] = 0;

            if (strstr((char*)read_buffer, "OK\r\n"))
            {
                printf("[%s]\r\n", read_buffer);
                break;
            }

            // TODO set it back
            /*if (strstr((char*)read_buffer, "ERROR\r\n"))
            {
                printf("ERROR: [%s]\r\n", read_buffer);
                uint32_t elapsed_time = hal_timer_get_uticks() - start_time;
                printf("After: %d miliseconds \r\n", (int)(elapsed_time / 1000));
                return -1;
            }*/
        }

        uint32_t current_time = hal_timer_get_uticks();
		if (current_time < start_time)
        {
            printf("strange timestamp\r\n");
            return -3;
        }
		if ((current_time - start_time) > timeout_us)
        {
            printf("timeout. el_count: %d \r\n", read_buffer_el_count);
            printf("[%s]\r\n", read_buffer);
            return -2;
        }
    }

//    // Now wait for SRING
//    uint32_t start_sring_time = hal_timer_get_uticks();
//    read_buffer_el_count = 0;
//    for(;;)
//    {
//        uint32_t readable = cyhal_uart_readable(&telit_uart_obj);
//        if (readable > 0)
//        {
//            uint16_t remaining_size = BUFFER_SIZE - read_buffer_el_count;
//            uint16_t toread = (readable > remaining_size)? remaining_size : readable;
//
//            // Buffer is full...
//            if (toread == 0)
//            {
//                printf("Buffer is full: %s . Readable: %lu \r\n", read_buffer, readable);
//                return -4;
//            }
//
//            int retval = hal_uart_read(&read_buffer[read_buffer_el_count], toread);
//            if (retval < 0)
//            {
//                printf("hal_uart_read error\r\n");
//                return -1;
//            }
//            read_buffer_el_count += toread;
//
//            read_buffer[read_buffer_el_count] = 0;
//
//            if (strstr((char*)read_buffer, "SRING: 1\r\n"))
//            {
//            	uint32_t current_time = hal_timer_get_uticks();
//            	uint32_t elapsed_time_1 = current_time - start_time;
//            	uint32_t elapsed_time_2 = current_time - start_sring_time;
//            	printf("Got SRING after %d ms or %d ms\r\n", (int)(elapsed_time_1/1000), (int)(elapsed_time_2/1000));
//                printf("[%s]\r\n", read_buffer);
//                break;
//            }
//
//            // TODO set it back
//            /*if (strstr((char*)read_buffer, "ERROR\r\n"))
//            {
//                printf("ERROR: [%s]\r\n", read_buffer);
//                uint32_t elapsed_time = hal_timer_get_uticks() - start_time;
//                printf("After: %d miliseconds \r\n", (int)(elapsed_time / 1000));
//                return -1;
//            }*/
//        }
//
//        uint32_t current_time = hal_timer_get_uticks();
//		if (current_time < start_time)
//        {
//            printf("strange timestamp\r\n");
//            return -3;
//        }
//		if ((current_time - start_time) > timeout_us)
//        {
//            printf("Did not receive SRING timeout. el_count: %d \r\n", read_buffer_el_count);
//            return -2;
//        }
//    }

    return 0;
}

int rab_rtk_telit_socket_read(uint8_t connection_id, uint16_t buffer_len, uint8_t* buffer)
{
	const uint16_t telit_max_receive = 1500; // Defined in data sheet page 1335/2014
	const uint8_t conn_id = 1; // TODO make it configurable
	char cmd[32] = {0};

	// First, get amount of data inside the buffer
	uint16_t sent_size = 0;
	uint16_t received_size = 0;
	uint16_t buff_in_size = 0;
	uint16_t toread = 0;

	int retval = rab_rtk_telit_get_socket_information(&sent_size, &received_size, &buff_in_size);
	if (retval != 0)
	{
#ifdef DEBUG_OUTPUT
        printf("rab_rtk_telit_get_socket_information error: %d\r\n", retval);
#endif
        return -1;
	}

	if (buff_in_size < buffer_len) toread = buff_in_size; // Less to read in buffer as requested
	else toread = buffer_len;

#ifdef DEBUG_OUTPUT
	printf("buffer_len : %d, buff_in_size: %d, toread = %d \r\n", buffer_len, buff_in_size, toread);
#endif

	uint16_t read_address = 0;

	for(;;)
	{
		if (toread == 0) break;

		uint16_t chunk_read = toread;
		if (chunk_read > telit_max_receive) chunk_read = telit_max_receive;

		// Read
		sprintf(cmd, "AT#SRECV=%d,%d\r\n", conn_id, chunk_read);
		int retval = send_command_and_wait_rcv(cmd, chunk_read, &buffer[read_address], STANDARD_TIMEOUT_US);
		if (retval != 0)
		{
	#ifdef DEBUG_OUTPUT
			printf("send_command_and_wait_rcv %d \r\n", retval);
	#endif
			return -1;
		}

		toread -= chunk_read;
		read_address += chunk_read;
	}

	return 0;
}

int rab_rtk_telit_socket_write(uint8_t connection_id, uint16_t buffer_len, uint8_t* buffer)
{
    uint32_t timeout_us = 1000000;

    read_available();

    // Construct the command to be sent
    char cmd[64] = {0};
    sprintf(cmd, "AT#SSENDEXT=%d,%d\r\n", connection_id, buffer_len);

    // Send the command
    size_t len = strlen(cmd);
    cyhal_uart_write(&telit_uart_obj, cmd, &len);

#ifdef DEBUG_OUTPUT
    printf("Cmd is: %s \r\n", cmd);
#endif

    wait_for_socket_input();

    // Send the buffer
    size_t buff_len = (size_t) buffer_len;
    cyhal_uart_write(&telit_uart_obj, buffer, &buff_len);

    read_buffer_el_count = 0;
    uint32_t start_time = hal_timer_get_uticks();
    for(;;)
    {
        uint32_t readable = cyhal_uart_readable(&telit_uart_obj);
        if (readable > 0)
        {
            uint16_t remaining_size = BUFFER_SIZE - read_buffer_el_count;
            uint16_t toread = (readable > remaining_size)? remaining_size : readable;

            // Buffer is full...
            if (toread == 0)
            {
                printf("Buffer is full: %s . Readable: %lu \r\n", read_buffer, readable);
                return -4;
            }

            int retval = hal_uart_read(&read_buffer[read_buffer_el_count], toread);
            if (retval < 0)
            {
                printf("hal_uart_read error\r\n");
                return -1;
            }
            read_buffer_el_count += toread;

            read_buffer[read_buffer_el_count] = 0;

            if (strstr((char*)read_buffer, "OK\r\n"))
            {
                printf("[%s]\r\n", read_buffer);
                return 0;
            }

            if (strstr((char*)read_buffer, "ERROR\r\n"))
            {
                printf("ERROR: [%s]\r\n", read_buffer);
                uint32_t elapsed_time = hal_timer_get_uticks() - start_time;
                printf("After: %d miliseconds \r\n", (int)(elapsed_time / 1000));
                return -1;
            }
        }

        uint32_t current_time = hal_timer_get_uticks();
		if (current_time < start_time)
        {
            printf("strange timestamp\r\n");
            return -3;
        }
		if ((current_time - start_time) > timeout_us)
        {
            printf("timeout. el_count: %d \r\n", read_buffer_el_count);
            printf("[%s]\r\n", read_buffer);
            return -2;
        }
    }
    return 0;
}
