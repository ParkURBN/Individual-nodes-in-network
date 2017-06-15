/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application. 
 * Public key was updated to validate the building precess.
 * Debugging with LPC LINK 2, SWD
 */

#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "app_util_platform.h"

#include "sdk_config.h"
#include "ble_advdata.h"
#include "ble_gap.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_uart.h"
#include "nrf_drv_twi.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "SEGGER_RTT.h"

#include "app_error.h"
#include "nrf.h"
#include "bsp.h"
#include "app_uart.h"
#include "app_fifo.h"
#include "nrf_drv_uart.h"
#include "nrf_assert.h"
#include "sdk_common.h"


#define CENTRAL_LINK_COUNT              0                                 /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           0                                 /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                 /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define APP_CFG_NON_CONN_ADV_TIMEOUT    0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH          0x17                              /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                              /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                              /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                              /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                            /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                        /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04                        /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */

#define DEAD_BEEF                       0xDEADBEEF                        /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_TIMER_PRESCALER             0                                 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                 /**< Size of timer operation queues. */

//--------------------------------------------------------------------------------------------//\
//UART
#define MAX_TEST_DATA_BYTES     (20U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                           /**< UART RX buffer size. */

#ifndef APP_UART_DRIVER_INSTANCE
#define APP_UART_DRIVER_INSTANCE 0

//I2C

/* TWI instance ID. */
#define TWI_INSTANCE_ID            0
#define TWI0_USE_EASY_DMA 0

/* Common addresses definition for temperature sensor. */
#define MMC3416_ADDR               0x60

#define MMC3416_REG_Xout_Low       0x00
#define MMC3416_REG_Xout_High      0x01
#define MMC3416_REG_Yout_Low       0x02
#define MMC3416_REG_Yout_High      0x03
#define MMC3416_REG_Zout_Low       0x04
#define MMC3416_REG_Zout_High      0x05
#define MMC3416_REG_Internal_CRL0   0x07
#define MMC3416_REG_Internal_CRL1   0x08
                                          
#endif

void DataTX(uint8_t * p_data);
static nrf_drv_uart_t app_uart_inst = NRF_DRV_UART_INSTANCE(APP_UART_DRIVER_INSTANCE);

static __INLINE uint32_t fifo_length(app_fifo_t * const fifo)
{
  uint32_t tmp = fifo->read_pos;
  return fifo->write_pos - tmp;
}

#define FIFO_LENGTH(F) fifo_length(&F)              /**< Macro to calculate length of a FIFO. */
//#define FIFO_LENGTH() fifo_length(p_fifo)           /**< Macro for calculating the FIFO length. */


static app_uart_event_handler_t   m_event_handler;            /**< Event handler function. */
static uint8_t tx_buffer[1];
static uint8_t rx_buffer[1];
static bool m_rx_ovf;

static app_fifo_t                  m_rx_fifo;                               /**< RX FIFO buffer for storing data received on the UART until the application fetches them using app_uart_get(). */
static app_fifo_t                  m_tx_fifo;    

//------------------------------ pragmas dont belong here----------------------------------//

#define SCAN_INTERVAL             0x0320                                   /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW               0x0190                                   /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT              0x0000                                   /**< Timout when scanning. 0x0000 disables timeout. */
#define SCAN_REQUEST              0                                        /**< Active scanning is not set. */
#define SCAN_WHITELIST_ONLY       0                                        /**< We will not ignore unknown devices. */
#define sensor_0  								8


//---------------------------------------------------------------------------------------//

#define LEDBUTTON_BUTTON_PIN      BSP_BUTTON_0
#define BUTTON_DETECTION_DELAY    APP_TIMER_TICKS(50, APP_TIMER_PRESCALER) /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define GPIOTE_CHANNEL_0 0

//------------  "Local made global" variables ---------------------------//

ble_advdata_manuf_data_t manuf_specific_data;
ble_advdata_t advdata; 

//---------------------------Dont belong here----------------------------//


static const ble_gap_scan_params_t m_scan_param =
{
    SCAN_REQUEST,
    SCAN_WHITELIST_ONLY,
    NULL,
    (uint16_t)SCAN_INTERVAL,
    (uint16_t)SCAN_WINDOW,
    (uint16_t)SCAN_TIMEOUT
};


/**@brief Variable length data encapsulation in terms of length and pointer to data. */
typedef struct
{
    uint8_t * p_data;   /**< Pointer to data. */
    uint16_t  data_len; /**< Length of data. */
} data_t;



bool ready_scan = false;
bool ready_trans = false;

static uint8_t m_sample[6]; 

unsigned char Internal_CRL0 = 0x00;//16 bit and 4ms
unsigned char Internal_CRL1 = 0z01;

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
		
static ble_gap_adv_params_t m_adv_params; 

uint8_t * p_data;
int length;
#define GPIO_TOGGLE_PIN  17       /*!< gpio pin to toggle after delay. */
data_t   adv_data;

//-----------------------------------------------------------------------//

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                        /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif


static ble_gap_adv_params_t m_adv_params;                                 /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                         // this implementation.
};

//--------------------------------CUSTOM FUNCTIONs----------------------------------------------//

//------------------------------Initializing LEDs-----------------------------------------//
void  LED_init(void){
		
	  LEDS_CONFIGURE(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK | BSP_LED_3_MASK);
    LEDS_OFF(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK | BSP_LED_3_MASK);
		nrf_gpio_cfg_sense_input(sensor_0, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);

}

void MMC3416_set_mode(void)
{
    ret_code_t err_code;

    uint8_t reg[2] = {Internal_CRL0, Internal_CRL1};	
    err_code = nrf_drv_twi_tx(&m_twi, MMC3416_ADDR, reg, sizeof(reg), false);
    //APP_ERROR_CHECK(err_code);
    //while (m_xfer_done == false);

}

/**@brief Put one byte to the FIFO. */
static __INLINE void fifo_put(app_fifo_t * p_fifo, uint8_t byte)
{
    p_fifo->p_buf[p_fifo->write_pos & p_fifo->buf_size_mask] = byte;
    p_fifo->write_pos++;
}


/**@brief Look at one byte in the FIFO. */
static __INLINE void fifo_peek(app_fifo_t * p_fifo, uint16_t index, uint8_t * p_byte)
{
    *p_byte = p_fifo->p_buf[(p_fifo->read_pos + index) & p_fifo->buf_size_mask];
}


/**@brief Get one byte from the FIFO. */
static __INLINE void fifo_get(app_fifo_t * p_fifo, uint8_t * p_byte)
{
    fifo_peek(p_fifo, 0, p_byte);
    p_fifo->read_pos++;
}


uint32_t app_fifo_init(app_fifo_t * p_fifo, uint8_t * p_buf, uint16_t buf_size)
{
    // Check buffer for null pointer.
    if (p_buf == NULL)
    {
        return NRF_ERROR_NULL;
    }

    // Check that the buffer size is a power of two.
    if (!IS_POWER_OF_TWO(buf_size))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    p_fifo->p_buf         = p_buf;
    p_fifo->buf_size_mask = buf_size - 1;
    p_fifo->read_pos      = 0;
    p_fifo->write_pos     = 0;

    return NRF_SUCCESS;
}


uint32_t app_fifo_put(app_fifo_t * p_fifo, uint8_t byte)
{
    if (fifo_length(p_fifo) <= p_fifo->buf_size_mask)
    {
        fifo_put(p_fifo, byte);
        return NRF_SUCCESS;
    }

    return NRF_ERROR_NO_MEM;
}


uint32_t app_fifo_get(app_fifo_t * p_fifo, uint8_t * p_byte)
{
    if (fifo_length(p_fifo) != 0)
    {
        fifo_get(p_fifo, p_byte);
        return NRF_SUCCESS;
    }

    return NRF_ERROR_NOT_FOUND;

}


uint32_t app_fifo_peek(app_fifo_t * p_fifo, uint16_t index, uint8_t * p_byte)
{
    if (fifo_length(p_fifo) > index)
    {
        fifo_peek(p_fifo, index, p_byte);
        return NRF_SUCCESS;
    }

    return NRF_ERROR_NOT_FOUND;
}


uint32_t app_fifo_flush(app_fifo_t * p_fifo)
{
    p_fifo->read_pos = p_fifo->write_pos;
    return NRF_SUCCESS;
}


uint32_t app_fifo_read(app_fifo_t * p_fifo, uint8_t * p_byte_array, uint32_t * p_size)
{
    VERIFY_PARAM_NOT_NULL(p_fifo);
    VERIFY_PARAM_NOT_NULL(p_size);

    const uint32_t byte_count    = fifo_length(p_fifo);
    const uint32_t requested_len = (*p_size);
    uint32_t       index         = 0;
    uint32_t       read_size     = MIN(requested_len, byte_count);

    (*p_size) = byte_count;

    // Check if the FIFO is empty.
    if (byte_count == 0)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    // Check if application has requested only the size.
    if (p_byte_array == NULL)
    {
        return NRF_SUCCESS;
    }

    // Fetch bytes from the FIFO.
    while (index < read_size)
    {
        fifo_get(p_fifo, &p_byte_array[index++]);
    }

    (*p_size) = read_size;

    return NRF_SUCCESS;
}


uint32_t app_fifo_write(app_fifo_t * p_fifo, uint8_t const * p_byte_array, uint32_t * p_size)
{
    VERIFY_PARAM_NOT_NULL(p_fifo);
    VERIFY_PARAM_NOT_NULL(p_size);

    const uint32_t available_count = p_fifo->buf_size_mask - fifo_length(p_fifo) + 1;
    const uint32_t requested_len   = (*p_size);
    uint32_t       index           = 0;
    uint32_t       write_size      = MIN(requested_len, available_count);

    (*p_size) = available_count;

    // Check if the FIFO is FULL.
    if (available_count == 0)
    {
        return NRF_ERROR_NO_MEM;
    }

    // Check if application has requested only the size.
    if (p_byte_array == NULL)
    {
        return NRF_SUCCESS;
    }

    //Fetch bytes from the FIFO.
    while (index < write_size)
    {
        fifo_put(p_fifo, p_byte_array[index++]);
    }

    (*p_size) = write_size;

    return NRF_SUCCESS;
}

//---------------------------------------------------------------------------------------//
static void uart_event_handler(nrf_drv_uart_event_t * p_event, void* p_context)
{
    app_uart_evt_t app_uart_event;
    uint32_t err_code;

    switch (p_event->type)
    {
        case NRF_DRV_UART_EVT_RX_DONE:
            // Write received byte to FIFO.
            err_code = app_fifo_put(&m_rx_fifo, p_event->data.rxtx.p_data[0]);
            if (err_code != NRF_SUCCESS)
            {
                app_uart_event.evt_type          = APP_UART_FIFO_ERROR;
                app_uart_event.data.error_code   = err_code;
                m_event_handler(&app_uart_event);
            }
            // Notify that there are data available.
            else if (FIFO_LENGTH(m_rx_fifo) != 0)
            {
                app_uart_event.evt_type = APP_UART_DATA_READY;
                m_event_handler(&app_uart_event);
            }

            // Start new RX if size in buffer.
            if (FIFO_LENGTH(m_rx_fifo) <= m_rx_fifo.buf_size_mask)
            {
                (void)nrf_drv_uart_rx(&app_uart_inst, rx_buffer, 1);
            }
            else
            {
                // Overflow in RX FIFO.
                m_rx_ovf = true;
            }

            break;

        case NRF_DRV_UART_EVT_ERROR:
            app_uart_event.evt_type                 = APP_UART_COMMUNICATION_ERROR;
            app_uart_event.data.error_communication = p_event->data.error.error_mask;
            (void)nrf_drv_uart_rx(&app_uart_inst, rx_buffer, 1);
            m_event_handler(&app_uart_event);
            break;

        case NRF_DRV_UART_EVT_TX_DONE:
            // Get next byte from FIFO.
            if (app_fifo_get(&m_tx_fifo, tx_buffer) == NRF_SUCCESS)
            {
                (void)nrf_drv_uart_tx(&app_uart_inst, tx_buffer, 1);
            }
            else
            {
                // Last byte from FIFO transmitted, notify the application.
                app_uart_event.evt_type = APP_UART_TX_EMPTY;
                m_event_handler(&app_uart_event);
            }
            break;

        default:
            break;
    }
}

uint32_t app_uart_init(const app_uart_comm_params_t * p_comm_params,
                             app_uart_buffers_t *     p_buffers,
                             app_uart_event_handler_t event_handler,
                             app_irq_priority_t       irq_priority)
{
    uint32_t err_code;

    m_event_handler = event_handler;

    if (p_buffers == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    // Configure buffer RX buffer.
    err_code = app_fifo_init(&m_rx_fifo, p_buffers->rx_buf, p_buffers->rx_buf_size);
    VERIFY_SUCCESS(err_code);

    // Configure buffer TX buffer.
    err_code = app_fifo_init(&m_tx_fifo, p_buffers->tx_buf, p_buffers->tx_buf_size);
    VERIFY_SUCCESS(err_code);

    nrf_drv_uart_config_t config = NRF_DRV_UART_DEFAULT_CONFIG;
    config.baudrate = (nrf_uart_baudrate_t)p_comm_params->baud_rate;
    config.hwfc = (p_comm_params->flow_control == APP_UART_FLOW_CONTROL_DISABLED) ?
            NRF_UART_HWFC_DISABLED : NRF_UART_HWFC_ENABLED;
    config.interrupt_priority = irq_priority;
    config.parity = p_comm_params->use_parity ? NRF_UART_PARITY_INCLUDED : NRF_UART_PARITY_EXCLUDED;
    config.pselcts = p_comm_params->cts_pin_no;
    config.pselrts = p_comm_params->rts_pin_no;
    config.pselrxd = p_comm_params->rx_pin_no;
    config.pseltxd = p_comm_params->tx_pin_no;

    err_code = nrf_drv_uart_init(&app_uart_inst, &config, uart_event_handler);
    VERIFY_SUCCESS(err_code);
    m_rx_ovf = false;

    // Turn on receiver if RX pin is connected
    if (p_comm_params->rx_pin_no != UART_PIN_DISCONNECTED)
    {
#ifdef UARTE_PRESENT
        if (!config.use_easy_dma)
#endif
        {
            nrf_drv_uart_rx_enable(&app_uart_inst);
        }

        return nrf_drv_uart_rx(&app_uart_inst, rx_buffer,1);
    }
    else
    {
        return NRF_SUCCESS;
    }
}

uint32_t app_uart_put(uint8_t byte)
{
    uint32_t err_code;
    err_code = app_fifo_put(&m_tx_fifo, byte);
    if (err_code == NRF_SUCCESS)
    {
        // The new byte has been added to FIFO. It will be picked up from there
        // (in 'uart_event_handler') when all preceding bytes are transmitted.
        // But if UART is not transmitting anything at the moment, we must start
        // a new transmission here.
        if (!nrf_drv_uart_tx_in_progress(&app_uart_inst))
        {
            // This operation should be almost always successful, since we've
            // just added a byte to FIFO, but if some bigger delay occurred
            // (some heavy interrupt handler routine has been executed) since
            // that time, FIFO might be empty already.
            if (app_fifo_get(&m_tx_fifo, tx_buffer) == NRF_SUCCESS)
            {
                err_code = nrf_drv_uart_tx(&app_uart_inst, tx_buffer, 1);
            }
        }
    }
    return err_code;
}

void uart_print(char *str)
{   
    //uint32_t err_code;
    int i = 0;
    while (str[i] != '\0'){
        while(app_uart_put(str[i])!= NRF_SUCCESS);
        i++;
    }
}

void uart_println(char *str)
{
    //uint32_t err_code;

    uart_print(str);
    while(app_uart_put('\n')!= NRF_SUCCESS);
}

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}


//----------------------------------------------------------------------------------------------//


void start_timer(void)
{		
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;  // Set the timer in Counter Mode
  NRF_TIMER2->TASKS_CLEAR = 1;               // clear the task first to be usable for later
	NRF_TIMER2->PRESCALER = 15;                             //Set prescaler. Higher number gives slower timer. Prescaler = 0 gives 16MHz timer
	NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;		 //Set counter to 16 bit resolution
	NRF_TIMER2->CC[0] = 500000;                             //Set value for TIMER2 compare register 0
	NRF_TIMER2->CC[1] = 500;                                 //Set value for TIMER2 compare register 1
		
  // Enable interrupt on Timer 2, both for CC[0] and CC[1] compare match events
	NRF_TIMER2->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos) | (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);
  NVIC_EnableIRQ(TIMER2_IRQn);
		
	NRF_TIMER2->TASKS_START = 1;               // Start TIMER2
	
  
}
		
/** TIMTER2 peripheral interrupt handler. This interrupt handler is called whenever there it a TIMER2 interrupt
 */
void TIMER2_IRQHandler(void)
{
	if ((NRF_TIMER2->EVENTS_COMPARE[0] != 0) && ((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE0_Msk) != 0))
  {
		NRF_TIMER2->EVENTS_COMPARE[0] = 0;           //Clear compare register 0 event	
		//nrf_gpio_pin_set(GPIO_TOGGLE_PIN);           //Set LED
		//LEDS_ON(BSP_LED_1_MASK);
		
		
		//-----------------------the scanning function--------------------------------------//
	 // sd_ble_gap_scan_stop();
		//SEGGER_RTT_printf(0,"Inside the 1st timer loop- 1\n");
		//sd_ble_gap_scan_start(&m_scan_param);
		LEDS_ON(BSP_LED_1_MASK);
		if (ready_scan == false){
			ready_scan = true;
			ready_trans = false;}
		else if (ready_scan == true){
			ready_scan = false;
			ready_trans = true;
		}
		//SEGGER_RTT_printf(0,"Inside the 1st timer loop- 2\n");
		//nrf_delay_ms(600);
	//	sd_ble_gap_scan_stop();
		
		
  }
	
	if ((NRF_TIMER2->EVENTS_COMPARE[1] != 0) && ((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE1_Msk) != 0))
  {
		NRF_TIMER2->EVENTS_COMPARE[1] = 0;           //Clear compare register 1 event
		//nrf_gpio_pin_clear(GPIO_TOGGLE_PIN);         //Clear LED
		LEDS_OFF(BSP_LED_1_MASK);
		//sd_ble_gap_scan_stop();
		//SEGGER_RTT_printf(0,"Inside the 2nd timer loop\n");
  }
}



static void on_adv_report(const ble_evt_t * const p_ble_evt)
{
    //uint32_t err_code;
    
    //bool     do_connect = false;
	  bool 		 checksum_correct = false;
    //data_t   dev_name;
		int sum = 0, checksum = 0;
    uint8_t * trial_data;
    // For readibility.
    const ble_gap_evt_t * const  p_gap_evt  = &p_ble_evt->evt.gap_evt;
    //const ble_gap_addr_t * const peer_addr  = &p_gap_evt->params.adv_report.peer_addr;
    {    
        // Initialize advertisement report for parsing
			//if (p_gap_evt->params.adv_report.data[5] == 01){
        adv_data.p_data     = (uint8_t *)p_gap_evt->params.adv_report.data;
        adv_data.data_len   = p_gap_evt->params.adv_report.dlen;
      

			//for(int i = 0; i < (adv_data.data_len - 1); i++)
				//		sum = adv_data.p_data[i] + sum; 
			
			if (adv_data.p_data[5] == 0x03 &&  adv_data.p_data[6] == 0x11){// && adv_data.p_data[29] == sum%256){
				//SEGGER_RTT_printf(0,"b scanning - 0x%x\n",adv_data.p_data[9]);
				trial_data = adv_data.p_data;
				
				for(int i = 5; i < 30; i++)
					sum = sum + trial_data[i];
				
				checksum = sum % 127;
				if (checksum == trial_data[30])
					checksum_correct = true;
				else 
					checksum_correct = false;
				
			  if (checksum_correct){
						p_data = adv_data.p_data;
						length = adv_data.data_len;
				}
			}
        //search for advertising names
        //bool found_name = false;
       /* err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
                                    &adv_data,
                                    &dev_name);*/
       /* if (err_code != NRF_SUCCESS)
        {
            // Look for the short local name if it was not found as complete
            err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
                                        &adv_data,
                                        &dev_name);
            if (err_code != NRF_SUCCESS)
            {
                // If we can't parse the data, then exit
                return;
            }
            else
            {
                found_name = true;
            }
        }
        else
        {
            found_name = true;
        }*/
       /* if (found_name)
        {
            if (strlen(m_target_periph_name) != 0)
            {
                if(memcmp(m_target_periph_name, dev_name.p_data, dev_name.data_len )== 0)
                {
                    do_connect = true;
                }
            }
        }*/
    }

   /* if (do_connect)
    {
        // Initiate connection.
        err_code = sd_ble_gap_connect(peer_addr, &m_scan_param, &m_connection_param);
        APP_ERROR_CHECK(err_code);
    }
		*/
}

/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_scan_stop();
    // It is okay to ignore this error since we are stopping the scan anyway.
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }

    err_code = sd_ble_gap_scan_start(&m_scan_param);
    APP_ERROR_CHECK(err_code);

   // LEDS_OFF(CENTRAL_CONNECTED_LED);
    //LEDS_ON(CENTRAL_SCANNING_LED);
}

static void on_ble_evt(const ble_evt_t * const p_ble_evt)
{
    // For readability.
    const ble_gap_evt_t * const p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected (HR or RSC), initiate DB
        // discovery, update LEDs status and resume scanning if necessary. */
        case BLE_GAP_EVT_CONNECTED:
        {
            //uint32_t err_code;

           // err_code = ble_lbs_c_handles_assign(&m_ble_lbs_c, p_gap_evt->conn_handle, NULL);
            //APP_ERROR_CHECK(err_code);

            //err_code = ble_db_discovery_start(&m_ble_db_discovery, p_gap_evt->conn_handle);
            //APP_ERROR_CHECK(err_code);

            // Update LEDs status, and check if we should be looking for more
            // peripherals to connect to.
            //LEDS_ON(CENTRAL_CONNECTED_LED);
            //LEDS_OFF(CENTRAL_SCANNING_LED);
        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            scan_start();
        } break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_ADV_REPORT:
            on_adv_report(p_ble_evt);
            break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                //NRF_LOG_PRINTF("[APP]: Connection Request timed out.\r\n");
            }
        } break; // BLE_GAP_EVT_TIMEOUT

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            ret_code_t err_code;
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break; // BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}



static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    on_ble_evt(p_ble_evt);
   // ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
    //ble_lbs_c_on_ble_evt(&m_ble_lbs_c, p_ble_evt);

}




/**@brief Function for handling BLE Stack events concerning central applications.
 *
 * @details This function keeps the connection handles of central applications up-to-date. It
 * parses scanning reports, initiating a connection attempt to peripherals when a target UUID
 * is found, and manages connection parameter update requests. Additionally, it updates the status
 * of LEDs used to report central applications activity.
 *
 * @note Since this function updates connection handles, @ref BLE_GAP_EVT_DISCONNECTED events
 *       should be dispatched to the target application before invoking this function.
 *
 * @param[in] p_ble_evt Bluetooth stack event.
 */



//------------------------------------------------------------------------------//
/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    //ble_advdata_t advdata;                  // made global
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

   // ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
    // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
    // UICR instead of using the default values. The major and minor values obtained from the UICR
    // are encoded into advertising data in big endian order (MSB First).
    // To set the UICR used by this example to a desired value, write to the address 0x10001080
    // using the nrfjprog tool. The command to be used is as follows.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
    // For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
    // the following command should be used.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
    uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
    uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

    uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;

    m_beacon_info[index++] = MSB_16(major_value);
    m_beacon_info[index++] = LSB_16(major_value);
    //m_beacon_info[9] = 0x55;
		//m_beacon_info[10] = 0x55;
    m_beacon_info[index++] = MSB_16(minor_value);
    m_beacon_info[index++] = LSB_16(minor_value);
#endif

    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;//0x01;
    advdata.p_manuf_specific_data = &manuf_specific_data;
//advdata.p_manuf_specific_data->data[1] = 0x51;
    
    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
	
	  err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
/*static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    uint32_t err_code;
    switch (pin_no)
    {
        case LEDBUTTON_BUTTON_PIN:
            manuf_specific_data.company_identifier = 0x51;
				    LEDS_ON(BSP_LED_1_MASK);
            break;
        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}*/






#ifdef BSP_BUTTON_0
    #define PIN_IN BSP_BUTTON_0
#endif
#ifndef PIN_IN
    #error "Please indicate input pin"
#endif

#ifdef BSP_LED_1
    #define PIN_OUT BSP_LED_1
#endif
#ifndef PIN_OUT
    #error "Please indicate output pin"
#endif

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    nrf_drv_gpiote_out_toggle(PIN_OUT);
	  LEDS_ON(BSP_LED_2_MASK);
	  manuf_specific_data.company_identifier = 0x51;
	  advdata.p_manuf_specific_data = &manuf_specific_data;

    ble_advdata_set(&advdata, NULL);
    //APP_ERROR_CHECK(err_code);
	  //advertising_start();
}

/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output, 
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_IN, true);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //data handler
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/**
 * @brief Function for reading data from temperature sensor.
 */
static void read_sensor_data(void)
{
    m_xfer_done = false;

    /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, MMC3416_ADDR, m_sample, 6);//6 bytes
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */

/**@brief Function for doing power management.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
          uint32_t err_code1;
	  uint32_t err_code;
	  uint32_t sw = 0;
	  int temp1,temp2,sum, checksum ;	
		
    // Initialize.
	  //------------------------------------------------------------------------//
	  //UART SETTINGS
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud9600
      };	

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code1);
			
    err_code1 = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code1);

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    err_code = bsp_init(BSP_INIT_LED, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
    ble_stack_init();
    gpio_init();	
    twi_init();
    MMC3416_set_mode();
    LED_init();
    start_timer(); 					
    advertising_init();

    // Start execution.
    NRF_LOG_INFO("BLE Beacon started\r\n");//Kept to avoid compilng errors
    advertising_start();

    // Enter main loop.
    for (;; )
    {
		    if (ready_scan)				
			      sd_ble_gap_scan_start(&m_scan_param);
				
		    if (!ready_scan)				
			      sd_ble_gap_scan_stop(); 
				
				sw = nrf_gpio_pin_read(sensor_0); // checking the input from sensor 
				
				//I2C sensor read
                                nrf_delay_ms(5);//5ms for 16 bit 4ms mode
				read_sensor_data();

        if (NRF_LOG_PROCESS() == false)//kept to avoid compiling errors
        {
            //power_manage();
        }

//---------------------- transmiting data over BLE ----------------------------------//				
			  if (ready_trans){
			 // manuf_specific_data.company_identifier = 0x69;//(p_data[5]+0x10);
				//m_beacon_info[2] = p_data[9];
				if(sw){
					m_beacon_info[0] =  (0x08 | p_data[7]) ;// 0x80;//
					LEDS_ON(BSP_LED_3_MASK);
				}
				else{
					m_beacon_info[0] =  (0xF7 & p_data[7]);// 0x00;//
					LEDS_OFF(BSP_LED_3_MASK);
				}	
	                
				//SEGGER_RTT_printf(0,"a scanning - 0x%x\n",p_data[7]);
				manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
			        advdata.p_manuf_specific_data = &manuf_specific_data;
				
				sum = 0;
				checksum = 0;
				temp1 = temp2 = (int16_t)(manuf_specific_data.company_identifier);
				temp1 = (temp1 >> 8);
				temp2 = (temp2 & 0xFF);
				
				for(int i = 0; i < 22; i++){
					sum = sum + manuf_specific_data.data.p_data[i];
				}
				sum = sum + temp1 + temp2; 
				checksum = sum % 128;
				manuf_specific_data.data.p_data[22] = checksum;
				SEGGER_RTT_printf(0,"--0x%x",manuf_specific_data.data.p_data[22]);
				
				advdata.p_manuf_specific_data = &manuf_specific_data;
			        ble_advdata_set(&advdata, NULL);
				//uart_println(test_str);
	    	}	
//---------------------- transmiting data over Optical Communication ----------------------------------//
        DataTX(m_sample);
        nrf_delay_ms(5);//5ms for 16 bit 4ms mode

//---------------------- receiving data over Optical Communication ----------------------------------//
        
        
    }
}


/**
 * @}
 */
