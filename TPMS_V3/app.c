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
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "app.h"
#include "em_iadc.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "gatt_db.h"

void iadc_init(void)
{
    // Enable clocks
    CMU_ClockEnable(cmuClock_IADC0, true);
    CMU_ClockEnable(cmuClock_GPIO, true);

    // Allocate A Bus even (for PA2) and odd (for PA1) to ADC0
    GPIO->ABUSALLOC = (1 << _GPIO_ABUSALLOC_AEVEN0_SHIFT)
                    | (1 << _GPIO_ABUSALLOC_AODD0_SHIFT);

    // Use default init structures
    IADC_Init_t        init        = IADC_INIT_DEFAULT;
    IADC_AllConfigs_t  allConfigs  = IADC_ALLCONFIGS_DEFAULT;
    IADC_InitSingle_t  initSingle  = IADC_INITSINGLE_DEFAULT;
    IADC_SingleInput_t singleInput = IADC_SINGLEINPUT_DEFAULT;

    // Set positive input: PA1
    singleInput.posInput = IADC_portPinToPosInput(gpioPortA, 1);

    // Set negative input: PA2
    singleInput.negInput = iadcNegInputGnd;

    // Initialize IADC
    IADC_init(IADC0, &init, &allConfigs);
    IADC_initSingle(IADC0, &initSingle, &singleInput);
}

static float iadc_read_temp(void)
{
  // Start conversion
  IADC_command(IADC0, iadcCmdStartSingle);

  // Wait for result
  while ((IADC0->STATUS & IADC_STATUS_SINGLEFIFODV) == 0);

  // Read result
  IADC_Result_t raw = IADC_pullSingleFifoResult(IADC0);
  uint32_t tempData = raw.data;

  float v = (tempData / 4095.0f) * 1210.0f; // 1.21 V internal reference, result in mV

  float temp = (v - 500.0f) / 10.0f; // Vout = Temp*Gain - Voffset

  return temp;
}

static uint8_t advertising_set_handle = 0xff;

static void update_advertisement(void)
{
    // Read System ID from GATT database (8 bytes)
    uint8_t sys_id_buf[8];
    uint16_t sys_id_len = 0;
    sl_bt_gatt_server_read_attribute_value(gattdb_device_name,
                                           0,
                                           sizeof(sys_id_buf),
                                           &sys_id_len,
                                           sys_id_buf);

    // Read temperature
    float temp = iadc_read_temp();
    int16_t temp_x10 = (int16_t)(temp * 10.0f); // e.g. 23.5°C → 235

    // Build advertisement payload:
    // [len][type=0xFF Manufacturer Specific][sys_id bytes][temp bytes]
    uint8_t adv_data[31];
    uint8_t idx = 0;

    // Manufacturer Specific Data (type 0xFF)
    // Length = 1 (type) + sys_id_len + 2 (temp)
    adv_data[idx++] = (uint8_t)(1 + sys_id_len + 2);
    adv_data[idx++] = 0xFF; // type: Manufacturer Specific
    for (size_t i = 0; i < sys_id_len && idx < 29; i++) {
        adv_data[idx++] = sys_id_buf[i];
    }
    adv_data[idx++] = (uint8_t)(temp_x10 & 0xFF);
    adv_data[idx++] = (uint8_t)((temp_x10 >> 8) & 0xFF);

    sl_bt_advertiser_set_data(advertising_set_handle, 0, idx, adv_data);
}

SL_WEAK void app_init(void)
{
    iadc_init();
}

SL_WEAK void app_process_action(void)
{
}

void sl_bt_on_event(sl_bt_msg_t *evt)
{
    sl_status_t sc;

    switch (SL_BT_MSG_ID(evt->header)) {

        case sl_bt_evt_system_boot_id:
            sc = sl_bt_advertiser_create_set(&advertising_set_handle);
            app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to create advertising set\n", (int)sc);

            sc = sl_bt_advertiser_set_timing(
                advertising_set_handle,
                160,  // min interval (100ms)
                160,  // max interval (100ms)
                0,    // duration: no time limit (controlled by maxevents instead)
                3);   // maxevents: send exactly 3 advertisement packets then stop
            app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to set advertising timing\n", (int)sc);

            sl_bt_system_set_soft_timer(32768 * 60, 0, 0);

            update_advertisement();
            sc = sl_bt_advertiser_start(
                advertising_set_handle,
                sl_bt_advertiser_general_discoverable,
                sl_bt_advertiser_non_connectable);
            app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to start advertising\n", (int)sc);
            break;

        case sl_bt_evt_system_soft_timer_id:
            // 60 seconds elapsed — read IADC and send 3-packet burst
            update_advertisement();
            sc = sl_bt_advertiser_start(
                advertising_set_handle,
                sl_bt_advertiser_general_discoverable,
                sl_bt_advertiser_non_connectable);
            app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to start advertising\n", (int)sc);
            break;

        default:
            break;
    }
}
