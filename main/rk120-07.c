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

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

void RK120_07_Init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static uint16_t s_direction = 0;
static float s_speed = 0;

#define BUFFER_SIZE 32

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            char *strLine[16] = {0};
            int numLine = 0;

            char *pch = NULL;
            pch = strtok((char *)&data[0], "\r\n");
            while(pch != NULL) {
                //printf("%s\n", pch);
                strLine[numLine++] = strdup(pch);
                if(numLine >= 16)
                    break;
                pch = strtok(NULL, "\r\n");
            }

            int i;
            for(i=0;i<numLine;i++) {
                char *strToken[16] = {0};
                int numToken = 0;
                
                printf("%s\n", strLine[i]);

                pch = strtok(strLine[i], ",");
                while(pch != NULL) {
                    printf("[%d] %s\n", numToken, pch);
                    strToken[numToken++] = strdup(pch);
                    if(numToken >= 16)
                        break;
                    pch = strtok(NULL, ",");
                }
                printf("\n");

                if(numToken == 6) {
                    if(strcmp(strToken[0], "$WIMWV") == 0 &&
                            strcmp(strToken[2], "R") == 0 &&
                            strncmp(strToken[5], "A*", 2) == 0) {
                        s_speed = atof(strToken[3]);
                        s_direction = (uint16_t)atof(strToken[1]);
                        printf("Wind speed %f, direction %u\n", s_speed, s_direction);
                    }
                }

                int j;
                for(j=0;j<numToken;j++) {
                    if(strToken[j] != 0)
                        free(strToken[j]);
                }
            }

            for(i=0;i<numLine;i++) {
                free(strLine[i]);
            }
        }
    }
    free(data);
}

void RK120_07_Run(void)
{
    xTaskCreate(rx_task, "uart_rx_task", 4096, NULL, configMAX_PRIORITIES, NULL);
}

uint16_t RK120_07_WindDirection()
{
    return s_direction;
}

float RK120_07_WindSpeed()
{
    return s_speed;
}
