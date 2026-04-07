/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
/***************************************************************************//**
 * @file
 * @brief Core application logic - Scanner with direct UART temperature output.
 ******************************************************************************/

// Capstone I16
// Written by Finn Rush
// PROTOTYPE

#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "app.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "gatt_db.h"
#include <string.h>
#include <stdio.h>

// UART peripheral and pins - configure to match your pintool assignment
#define UART_PERIPHERAL     USART0
#define UART_BAUD_RATE      115200
#define UART_TX_PORT        gpioPortA
#define UART_TX_PIN         1
#define UART_RX_PORT        gpioPortA
#define UART_RX_PIN         2

// Length of the full T###P# identifier in the advertisement payload
#define IDENTIFIER_LEN      6

// Offset and length of the P# portion within T###P#
#define PNUM_OFFSET         4
#define PNUM_LEN            2

// Local copy of this device's own name read from GATT (e.g. "T001P1 SPRAYER")
static uint8_t own_name[32];
static uint16_t own_name_len = 0;

static void uart_init(void)
{
    // Enable clocks
    CMU_ClockEnable(cmuClock_GPIO, true);
    CMU_ClockEnable(cmuClock_USART0, true);

    // Configure TX pin as push-pull output
    GPIO_PinModeSet(UART_TX_PORT, UART_TX_PIN, gpioModePushPull, 1);
    // Configure RX pin as input
    GPIO_PinModeSet(UART_RX_PORT, UART_RX_PIN, gpioModeInput, 0);

    // Initialize USART for async (UART) operation with defaults
    USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;
    init.baudrate = UART_BAUD_RATE;
    init.enable   = usartEnable;

    USART_InitAsync(UART_PERIPHERAL, &init);

    // Route USART TX and RX to the selected GPIO pins
    GPIO->USARTROUTE[0].TXROUTE = (UART_TX_PORT << _GPIO_USART_TXROUTE_PORT_SHIFT)
                                 | (UART_TX_PIN  << _GPIO_USART_TXROUTE_PIN_SHIFT);
    GPIO->USARTROUTE[0].RXROUTE = (UART_RX_PORT << _GPIO_USART_RXROUTE_PORT_SHIFT)
                                 | (UART_RX_PIN  << _GPIO_USART_RXROUTE_PIN_SHIFT);
    GPIO->USARTROUTE[0].ROUTEEN = GPIO_USART_ROUTEEN_TXPEN | GPIO_USART_ROUTEEN_RXPEN;
}

static void uart_print(const char *str)
{
    while (*str) {
        USART_Tx(UART_PERIPHERAL, (uint8_t)*str++);
    }
}

SL_WEAK void app_init(void)
{
    uart_init();
}

SL_WEAK void app_process_action(void)
{
}

void sl_bt_on_event(sl_bt_msg_t *evt)
{
    sl_status_t sc;

    switch (SL_BT_MSG_ID(evt->header)) {

        case sl_bt_evt_system_boot_id:
            // Read this device's own name from its local GATT database
            sl_bt_gatt_server_read_attribute_value(gattdb_device_name,
                                                   0,
                                                   sizeof(own_name),
                                                   &own_name_len,
                                                   own_name);

            // Set scan parameters: passive scan, 10ms interval, 10ms window
            sc = sl_bt_scanner_set_parameters(
                sl_bt_scanner_scan_mode_passive,
                16,
                16);
            app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to set scan parameters\n", (int)sc);

            sc = sl_bt_scanner_start(
                sl_bt_scanner_scan_phy_1m,
                sl_bt_scanner_discover_generic);
            app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to start scanning\n", (int)sc);
            break;

        case sl_bt_evt_scanner_scan_report_id: {
            uint8_t *data = evt->data.evt_scanner_scan_report.data.data;
            uint8_t  len  = evt->data.evt_scanner_scan_report.data.len;

            if (own_name_len < (PNUM_OFFSET + PNUM_LEN)) break;

            uint8_t i = 0;
            while (i < len) {
                uint8_t ad_len  = data[i];
                uint8_t ad_type = data[i + 1];

                if (ad_type == 0xFF && ad_len >= 9) {
                    uint8_t *payload = &data[i + 2];

                    // Compare only the P# portion of the identifier
                    if (memcmp(&payload[PNUM_OFFSET],
                               &own_name[PNUM_OFFSET],
                               PNUM_LEN) == 0) {

                        int16_t temp_x10 = (int16_t)(payload[6] | (payload[7] << 8));
                        float temp = temp_x10 / 10.0f;

                        // Format and send over UART
                        char buf[32];
                        snprintf(buf, sizeof(buf), "%.1f\r\n", temp);
                        uart_print(buf);
                    }
                }
                i += ad_len + 1;
            }
            break;
        }

        default:
            break;
    }
}
