/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp32/rom/gpio.h"
#include "esp32/rom/ets_sys.h"
#include "dywds2.h"

#include <string.h>
#include "atox.h"

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

void Dywds2_Init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static uint16_t s_direction = 0;
static float s_speed = 0;

#define BUFFER_SIZE 32

static uint8_t LRC(uint8_t *auchMsg, uint16_t usDataLen)
{
    uint8_t uchLRC = 0;                         // LRC 初始化
    while (usDataLen--)                           //完成整个报文缓冲区
        uchLRC += *auchMsg++;                           //缓冲区字节相加，无进位
    return ((uint8_t)(-((char)uchLRC))) ;   //返回二进制补码
}

const char itohex(uint8_t v)
{
  if(v < 10)
    return (v + '0');
  else
    return (v - 10) + 'A';
}

static int mbreq(const uint8_t addr, const uint8_t func, const uint8_t *data, uint8_t len)
{
    uint8_t raw[BUFFER_SIZE << 1];
    memset(raw, 0, BUFFER_SIZE << 1);

    char str[BUFFER_SIZE];
    memset(str, 0, BUFFER_SIZE);
    char *p = str;

    raw[0] = addr;
    raw[1] = func;
    memcpy(raw+2, data, len);
      
    *p++ = ':';
    *p++ = itohex(addr >> 4);
    *p++ = itohex(addr & 0xf);
    *p++ = itohex(func >> 4);
    *p++ = itohex(func & 0xf);
    
    uint8_t i;
    for(i=0;i<len;i++) {
        *p++ = itohex(data[i] >> 4);
        *p++ = itohex(data[i] & 0xf);
    }
    
    uint8_t lrc = LRC((uint8_t *)raw, len+2);

    *p++ = itohex(lrc >> 4);
    *p++ = itohex(lrc & 0xf);
    *p++ = '\r';
    *p++ = '\n';

    for(i=0;i<(p-str);i++)
        str[i] |= 0x80; /* 8N1 to 7N2 */

    return uart_write_bytes(UART_NUM_1, str, p-str);
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        const uint8_t data[] = {0x0, 0x0, 0x0, 0x3};
        mbreq(0x1, 0x3, data, 4);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            //ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            int i;
            for(i=0;i<rxBytes;i++) {
                data[i] &= 0x7f; // Only 7 bits are legal ...
            }
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

            uint8_t *p = &data[0];
            for(i=0;i<rxBytes;i++) {
                *p &= 0x7f;
                if(*p == ':') /* Start of MODBUS packet */
                    break;
                else
                    p++;
            }
            if(i == rxBytes)
                continue;
            p++; /* Ignore token ':' */
            uint8_t v = atohex8(p); /* addr */
            if(v == 1) { /* addr 1 - Wind speed*/
                s_direction = 0;
                s_speed = 0;

                p+=2;
                v = atohex8(p); /* Func */
                if(v != 3)
                    continue;
                p+=2;
                v = atohex8(p); /* Data length */
                p+=2;
                v = atohex8(p); /* Wind direction */
                s_direction = v << 8;
                p+=2;
                v = atohex8(p);
                s_direction |= v;
                p+=2;
                s_speed = atof32(p);

                printf("Wind speed %f, direction %u\n", s_speed, s_direction);
            }
        }
    }
    free(data);
}

void Dywds2_Run(void)
{
    xTaskCreate(rx_task, "uart_rx_task", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 2048, NULL, configMAX_PRIORITIES-1, NULL);
}

uint16_t Dywds2_WindDirection()
{
    return s_direction;
}

float Dywds2_WindSpeed()
{
    return s_speed;
}
